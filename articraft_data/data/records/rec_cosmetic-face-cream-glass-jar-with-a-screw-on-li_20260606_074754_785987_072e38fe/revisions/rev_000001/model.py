from __future__ import annotations

# Cosmetic FACE CREAM JAR (POND'S style).
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
# Squat blue-tinted glass jar (wider than tall) filled with a teal gel surface
# at the top, with a matching blue screw-on lid lifted above the neck.
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
JAR_OUTER_R = 0.036           # outer radius of the glass body (~0.072 m dia)
JAR_BODY_H = 0.044            # height of the glass body
WALL = 0.005                  # thick glass wall
NECK_R = 0.030                # outer radius of the threaded neck
NECK_H = 0.010                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim where the lid sits (0.054)

GEL_TOP_Z = JAR_BODY_H - 0.004    # teal gel surface sits just below the rim inside

LID_OUTER_R = 0.0375          # lid skirt slightly wider than the body
LID_H = 0.018                 # lid height (skirt depth + top)
# Lid geometry is authored in the lid part frame, whose origin coincides with
# the carrier/rotate joint at world z=RIM_TOP_Z. So lid-local z=0 is the rim top.
# The skirt bottom drops below the rim to slip over the threaded neck.
LID_SKIRT_BOTTOM_Z = -0.009   # lid-local: 9mm below the rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top of the cap (0.009)


def _jar_glass_solid() -> cq.Workplane:
    # Hollow thick-walled glass jar built as a revolve of a half profile in the
    # XZ plane (revolved about the Z axis). The profile traces the outer wall up,
    # across a rounded shoulder into the neck, then back down the inner wall to
    # form a real open-topped cavity.
    pts = [
        (0.0, 0.0),                          # center of the base
        (JAR_OUTER_R, 0.0),                  # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.006),   # outer wall up
        (JAR_OUTER_R - 0.004, JAR_BODY_H),   # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.002),        # step in to the neck
        (NECK_R, RIM_TOP_Z),                 # neck outer up to the rim
        (NECK_R - WALL, RIM_TOP_Z),          # across the rim top
        (NECK_R - WALL, JAR_BODY_H - 0.002), # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.006),
        (JAR_OUTER_R - WALL, WALL),          # inner body wall down to the thick base
        (0.0, WALL),                         # across the inner base
        (0.0, 0.0),                          # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    # A few thread ridges on the neck so it reads as a screw neck. Modeled as
    # thin stacked rings around the neck outer wall.
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


def _gel_surface_mesh():
    # Teal gel filling the jar to just below the rim, with a low domed top and a
    # small central swirl peak like a freshly opened gel.
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
    # Screw-on lid: a shallow cylindrical cup (closed crowned top + downward
    # skirt) that caps over the neck. Hollowed underneath so the skirt slips
    # over the neck.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    # Cavity matches the neck outer radius so the skirt inner wall seats directly
    # against the neck and its thread ridges, giving real contact when capped.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R)
        .extrude(LID_H - 0.005)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    # Knurled grip ring around the lid skirt: a band of vertical ribs.
    ribs = None
    n = 48
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
    band_h = LID_H - 0.005
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center((LID_OUTER_R - 0.0004) * math.cos(ang),
                    (LID_OUTER_R - 0.0004) * math.sin(ang))
            .rect(0.0016, 0.0016)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="face_cream_jar")

    glass_blue = model.material("glass_blue", rgba=(0.42, 0.66, 0.80, 0.55))
    gel_teal = model.material("gel_teal", rgba=(0.16, 0.74, 0.80, 1.0))
    lid_blue = model.material("lid_blue", rgba=(0.46, 0.72, 0.86, 1.0))
    label_white = model.material("label_white", rgba=(0.96, 0.97, 0.98, 1.0))
    marker_navy = model.material("marker_navy", rgba=(0.10, 0.18, 0.34, 1.0))

    # ---- jar body (root): glass shell + neck threads + teal gel + label ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_blue, name="jar_glass")

    # Teal gel filling the jar to just below the rim.
    body.visual(_gel_surface_mesh(), material=gel_teal, name="gel_surface")

    # Brand label: a thin white band wrapped on the front face of the body.
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0004, 0.018),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5 - 0.004)),
        material=label_white,
        name="brand_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.20,
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

    # ---- lid: caps over the neck; slides up off the carrier along +Z ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=lid_blue, name="lid_shell")
    lid.visual(_lid_knurl_mesh(), material=lid_blue, name="lid_knurl")
    # Small off-axis navy marker so rotation of the lid is visible.
    lid.visual(
        Box((0.004, 0.004, 0.002)),
        origin=Origin(xyz=(LID_OUTER_R - 0.006, 0.0, LID_SKIRT_BOTTOM_Z + LID_H - 0.001)),
        material=marker_navy,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_H),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_H * 0.5)),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_H, effort=1.0, velocity=1.0),
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
    body_pos = ctx.part_world_position(body)
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and body_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.001,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    # Lid caps the neck (projected footprint overlaps the body) at rest.
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid caps the neck"
    )

    # ---- lid_rotate spins the lid: the off-axis marker moves ----
    marker0 = ctx.part_element_world_aabb(lid, elem="lid_marker")
    m0 = ((marker0[0][0] + marker0[1][0]) * 0.5, (marker0[0][1] + marker0[1][1]) * 0.5)
    with ctx.pose({rotate: math.pi / 2.0}):
        marker1 = ctx.part_element_world_aabb(lid, elem="lid_marker")
        m1 = ((marker1[0][0] + marker1[1][0]) * 0.5, (marker1[0][1] + marker1[1][1]) * 0.5)
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
        # When fully lifted, the lid clears the neck rim along +Z.
        ctx.expect_gap(
            lid, body, axis="z", min_gap=0.0,
            positive_elem="lid_shell", negative_elem="jar_glass",
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

    return ctx.report()


object_model = build_object_model()
