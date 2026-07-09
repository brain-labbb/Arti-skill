from __future__ import annotations

# FACETED GLASS COSMETIC JAR with metal screw-on lid.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
# Octagonal (8-facet) glass jar body with thick walls, a round threaded neck,
# a base foot ring, and a rim seam ridge at the shoulder. Brushed-metal
# screw-on lid with knurled grip.
#
# Articulations (screw cap pattern, both share the vertical +Z axis, decoupled
# through a massless carrier link):
#   - lid_rotate: CONTINUOUS spin of the carrier about +Z at the rim top
#   - lid_slide:  PRISMATIC lift of the lid relative to the carrier along +Z

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
N_FACETS = 8
JAR_OUTER_R = 0.036           # circumscribed radius of the octagonal body
JAR_BODY_H = 0.044            # height of the main glass body
WALL = 0.004                  # glass wall thickness
NECK_R = 0.030                # outer radius of the round threaded neck
NECK_H = 0.010                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim where the lid seats

# Base foot ring — a thin octagonal ring protruding below the body base
FOOT_RING_R = JAR_OUTER_R * 0.80  # inscribed slightly inside the body
FOOT_RING_H = 0.002               # ring height below the body base

# Rim seam — thin outward ridge at the top of the body shoulder
RIM_SEAM_H = 0.0015
RIM_SEAM_PROTRUDE = 0.0012        # how far the seam protrudes past the facets

GEL_TOP_Z = JAR_BODY_H - 0.004    # teal gel surface sits just below the rim

LID_OUTER_R = 0.0375              # lid skirt slightly wider than the body
LID_H = 0.018                     # lid total height
LID_SKIRT_BOTTOM_Z = -0.009       # lid-local: skirt bottom below rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H


def _polygon_points(radius: float, n: int, angle_offset: float = 0.0):
    """Regular polygon vertices centered at origin in the XY plane."""
    return [
        (
            radius * math.cos(2.0 * math.pi * i / n + angle_offset),
            radius * math.sin(2.0 * math.pi * i / n + angle_offset),
        )
        for i in range(n)
    ]


def _faceted_jar_glass() -> cq.Workplane:
    """Octagonal hollow glass jar body with round threaded neck."""
    # Outer octagonal body
    outer_pts = _polygon_points(JAR_OUTER_R, N_FACETS)
    outer = (
        cq.Workplane("XY")
        .polyline(outer_pts)
        .close()
        .extrude(JAR_BODY_H)
    )

    # Hollow octagonal cavity (wall thickness offset from base and sides)
    inner_r = JAR_OUTER_R - WALL
    inner_pts = _polygon_points(inner_r, N_FACETS)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .polyline(inner_pts)
        .close()
        .extrude(JAR_BODY_H - WALL)
    )
    jar = outer.cut(cavity)

    # Round neck cylinder on top with matching wall thickness
    neck_outer = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .circle(NECK_R)
        .extrude(NECK_H)
    )
    neck_inner = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H - 0.001)
        .circle(NECK_R - WALL)
        .extrude(NECK_H + 0.002)
    )
    jar = jar.union(neck_outer).cut(neck_inner)

    return jar


def _neck_threads() -> cq.Workplane:
    """Thin thread ridges on the round neck."""
    threads = None
    z0 = JAR_BODY_H + 0.0035
    for i in range(3):
        z = z0 + i * 0.0028
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0006)
            .circle(NECK_R - 0.0004)
            .extrude(0.0016)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _base_foot_ring() -> cq.Workplane:
    """Octagonal foot ring protruding below the body base."""
    outer_pts = _polygon_points(FOOT_RING_R, N_FACETS)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-FOOT_RING_H)
        .polyline(outer_pts)
        .close()
        .extrude(FOOT_RING_H)
    )
    inner_r = FOOT_RING_R - WALL * 0.7
    inner_pts = _polygon_points(inner_r, N_FACETS)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-FOOT_RING_H - 0.001)
        .polyline(inner_pts)
        .close()
        .extrude(FOOT_RING_H + 0.002)
    )
    return outer.cut(inner)


def _rim_seam() -> cq.Workplane:
    """Thin outward-protruding octagonal ridge at the body shoulder."""
    seam_r = JAR_OUTER_R + RIM_SEAM_PROTRUDE
    outer_pts = _polygon_points(seam_r, N_FACETS)
    outer = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H - RIM_SEAM_H)
        .polyline(outer_pts)
        .close()
        .extrude(RIM_SEAM_H)
    )
    inner_pts = _polygon_points(JAR_OUTER_R - 0.001, N_FACETS)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H - RIM_SEAM_H - 0.001)
        .polyline(inner_pts)
        .close()
        .extrude(RIM_SEAM_H + 0.002)
    )
    return outer.cut(inner)


def _gel_surface_mesh():
    """Teal gel filling inside the jar."""
    inner_r = JAR_OUTER_R - WALL - 0.0008
    disc = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z - 0.012)
        .circle(inner_r)
        .extrude(0.012)
    )
    bulge = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.004)
        .circle(inner_r * 0.55)
        .loft(ruled=False)
    )
    peak = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z + 0.002)
        .circle(inner_r * 0.30)
        .workplane(offset=0.0035)
        .circle(0.0015)
        .loft(ruled=False)
    )
    gel = disc.union(bulge).union(peak)
    return mesh_from_cadquery(gel, "gel_surface")


def _lid_solid() -> cq.Workplane:
    """Metal screw-on lid: shallow cylindrical cup with cavity for the neck."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    # Cavity matches the neck outer radius
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R)
        .extrude(LID_H - 0.005)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    """Knurled grip ring around the lid skirt."""
    ribs = None
    n = 48
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
    band_h = LID_H - 0.005
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center(
                (LID_OUTER_R - 0.0004) * math.cos(ang),
                (LID_OUTER_R - 0.0004) * math.sin(ang),
            )
            .rect(0.0016, 0.0016)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_cosmetic_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.82, 0.88, 0.90, 0.45))
    gel_teal = model.material("gel_teal", rgba=(0.16, 0.74, 0.80, 1.0))
    metal_silver = model.material("metal_silver", rgba=(0.75, 0.76, 0.78, 1.0))
    marker_dark = model.material("marker_dark", rgba=(0.12, 0.12, 0.14, 1.0))

    # ---- jar body (root): faceted glass shell + threads + foot ring + rim seam + gel ----
    body = model.part("body")

    # Glass jar: faceted body + threads + foot ring + rim seam
    glass = (
        _faceted_jar_glass()
        .union(_neck_threads())
        .union(_base_foot_ring())
        .union(_rim_seam())
    )
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Teal gel filling
    body.visual(_gel_surface_mesh(), material=gel_teal, name="gel_surface")

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- massless carrier (no visuals): rotates about +Z at the rim top ----
    carrier = model.part("lid_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.008, 0.008, 0.008)), mass=1e-4)
    model.articulation(
        "lid_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    # ---- lid: metal screw-on cap over the neck ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=metal_silver,
        name="lid_shell",
    )
    lid.visual(_lid_knurl_mesh(), material=metal_silver, name="lid_knurl")
    # Off-axis marker so rotation is visible
    lid.visual(
        Box((0.004, 0.004, 0.002)),
        origin=Origin(
            xyz=(LID_OUTER_R - 0.006, 0.0, LID_SKIRT_BOTTOM_Z + LID_H - 0.001)
        ),
        material=marker_dark,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_H * 0.5)),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=LID_H, effort=1.0, velocity=1.0
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("lid_carrier")
    lid = object_model.get_part("lid")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")

    # The lid skirt seats over the neck rim; allow the small intentional overlap.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim.",
    )

    # ---- jar is squat: wider than it is tall ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.005 and bext[1] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.001,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid caps the neck"
    )

    # ---- lid_rotate is CONTINUOUS (screw joint) and spins the lid ----
    ctx.check(
        "lid_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )
    marker0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0 = (
        (marker0[0][0] + marker0[1][0]) * 0.5,
        (marker0[0][1] + marker0[1][1]) * 0.5,
    )
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1 = (
            (marker1[0][0] + marker1[1][0]) * 0.5,
            (marker1[0][1] + marker1[1][1]) * 0.5,
        )
    moved = math.hypot(m1[0] - m0[0], m1[1] - m0[1])
    ctx.check(
        "lid_rotate spins the lid (marker moves)",
        moved > 0.01,
        details=f"marker rest={m0}, quarter-turn={m1}, moved={moved}",
    )

    # ---- lid_slide lifts the lid off the jar ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H}):
        lifted_z = ctx.part_world_position(lid)[2]
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            positive_elem="lid_shell",
            negative_elem="jar_glass",
            name="lifted lid clears the neck",
        )
    ctx.check(
        "lid_slide lifts the lid off the jar",
        lifted_z > rest_z + LID_H * 0.5,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- faceted geometry: body X extent differs from Y at 45° check ----
    # An octagonal body should have similar X and Y extents (regular polygon)
    # but both should be less than a circumscribed circle diameter
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    circumscribed_dia = JAR_OUTER_R * 2.0
    ctx.check(
        "body has faceted (non-circular) proportions",
        body_dx < circumscribed_dia + 0.005 and body_dy < circumscribed_dia + 0.005,
        details=f"dx={body_dx}, dy={body_dy}, circumscribed={circumscribed_dia}",
    )

    # ---- foot ring geometry exists below the body base ----
    # The body AABB min-Z should extend below z=0 due to the foot ring
    ctx.check(
        "foot ring extends below body base",
        body_aabb[0][2] < -FOOT_RING_H * 0.5,
        details=f"body_aabb_min_z={body_aabb[0][2]}, expected < {-FOOT_RING_H * 0.5}",
    )

    # ---- rim seam protrudes beyond body facets ----
    # The body AABB in X or Y should be wider than just the octagon flat-to-flat
    # because the rim seam protrudes outward
    inscribed_dia = 2.0 * JAR_OUTER_R * math.cos(math.pi / N_FACETS)
    ctx.check(
        "rim seam protrudes beyond body facets",
        body_dx > inscribed_dia + RIM_SEAM_PROTRUDE * 0.5
        or body_dy > inscribed_dia + RIM_SEAM_PROTRUDE * 0.5,
        details=f"dx={body_dx}, dy={body_dy}, inscribed={inscribed_dia}",
    )

    return ctx.report()


object_model = build_object_model()
