from __future__ import annotations

# SQUAT COSMETIC CREAM JAR variant (wide-mouth, thick screw lid, gasket).
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Squat frosted-glass cream jar with a wide mouth, thick white screw-on lid,
# and a rubber gasket ring seated under the lid skirt. The mouth opening is
# prominently wider than the parent jar relative to body diameter.
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
JAR_OUTER_R = 0.042           # outer radius of the glass body (~0.084 m dia)
JAR_BODY_H = 0.034            # height of the glass body (squat)
WALL = 0.004                  # glass wall thickness
NECK_R = 0.038                # outer radius of the threaded neck (wide mouth)
NECK_H = 0.009                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim top (0.043)

MOUTH_R = NECK_R - WALL       # inner mouth radius (0.034) — wide opening
GEL_TOP_Z = JAR_BODY_H - 0.005   # cream surface sits just below the shoulder

LID_OUTER_R = 0.044           # lid skirt slightly wider than the body
LID_H = 0.026                 # thick lid (skirt depth + crowned top)
# Lid geometry is authored in the lid part frame, whose origin coincides with
# the carrier/rotate joint at world z=RIM_TOP_Z. So lid-local z=0 is the rim top.
LID_SKIRT_BOTTOM_Z = -0.012   # lid-local: 12mm below the rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top of the cap (0.014)

GASKET_R_MAJOR = NECK_R + 0.001   # gasket sits at neck outer radius
GASKET_R_MINOR = 0.0018           # gasket cross-section radius (tube)
GASKET_Z = LID_SKIRT_BOTTOM_Z + 0.001  # just above the skirt bottom, under the lid


def _jar_glass_solid() -> cq.Workplane:
    # Hollow thick-walled glass jar built as a revolve of a half profile in the
    # XZ plane (revolved about the Z axis). Wide mouth with gentle shoulder.
    pts = [
        (0.0, 0.0),                          # center of the base
        (JAR_OUTER_R, 0.0),                  # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.005),   # outer wall up
        (JAR_OUTER_R - 0.003, JAR_BODY_H),   # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.002),        # step in slightly to the neck
        (NECK_R, RIM_TOP_Z),                 # neck outer up to the rim
        (NECK_R - WALL, RIM_TOP_Z),          # across the rim top
        (NECK_R - WALL, JAR_BODY_H - 0.002), # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.005),
        (JAR_OUTER_R - WALL, WALL),          # inner body wall down to thick base
        (0.0, WALL),                         # across the inner base
        (0.0, 0.0),                          # close back to center
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    # Thread ridges on the wide neck so it reads as a screw neck.
    threads = None
    z0 = JAR_BODY_H + 0.003
    for i in range(3):
        z = z0 + i * 0.0025
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0006)
            .circle(NECK_R - 0.0004)
            .extrude(0.0014)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _cream_surface_mesh():
    # White cream filling the jar to just below the rim, with a smooth domed top.
    inner_r = MOUTH_R - 0.001
    disc = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z - 0.010)
        .circle(inner_r)
        .extrude(0.010)
    )
    dome = (
        cq.Workplane("XY")
        .workplane(offset=GEL_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.50)
        .loft(ruled=False)
    )
    cream = disc.union(dome)
    return mesh_from_cadquery(cream, "cream_surface")


def _lid_solid() -> cq.Workplane:
    # Thick screw-on lid: a cylindrical cup (closed crowned top + downward
    # skirt) that caps over the wide neck. Hollowed underneath.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.004)
    # Cavity matches neck outer radius so the skirt inner wall seats against
    # the neck and its thread ridges.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R)
        .extrude(LID_H - 0.006)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    # Knurled grip ring around the lid skirt: vertical ribs for grip.
    ribs = None
    n = 56
    band_z = LID_SKIRT_BOTTOM_Z + 0.003
    band_h = LID_H - 0.008
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center((LID_OUTER_R - 0.0004) * math.cos(ang),
                    (LID_OUTER_R - 0.0004) * math.sin(ang))
            .rect(0.0014, 0.0014)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def _gasket_mesh():
    # Rubber gasket ring seated under the lid, between the skirt bottom and
    # the jar rim. Modeled as a torus at the neck outer radius.
    # Build as a revolved circle cross-section.
    gasket = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .center(GASKET_R_MAJOR, GASKET_Z + GASKET_R_MINOR)
        .circle(GASKET_R_MINOR)
        .revolve(360.0, (0, 0, 0), (0, 0, 1))
    )
    return mesh_from_cadquery(gasket, "gasket_ring")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_cream_jar")

    glass_frost = model.material("glass_frost", rgba=(0.85, 0.88, 0.90, 0.60))
    cream_white = model.material("cream_white", rgba=(0.97, 0.95, 0.90, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.96, 1.0))
    gasket_rubber = model.material("gasket_rubber", rgba=(0.22, 0.22, 0.24, 1.0))
    label_silver = model.material("label_silver", rgba=(0.78, 0.80, 0.82, 1.0))

    # ---- jar body (root): glass shell + neck threads + cream + label ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_frost, name="jar_glass")

    # Cream filling the jar to just below the rim.
    body.visual(_cream_surface_mesh(), material=cream_white, name="cream_surface")

    # Brand label: a thin silver band on the front of the body.
    body.visual(
        Cylinder(JAR_OUTER_R + 0.0004, 0.014),
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.45)),
        material=label_silver,
        name="brand_label",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.25,
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

    # ---- lid: thick cap over the neck; slides up off the carrier along +Z ----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_solid(), "lid_shell"), material=lid_white, name="lid_shell")
    lid.visual(_lid_knurl_mesh(), material=lid_white, name="lid_knurl")

    # Gasket ring under the lid — moves with the lid when unscrewed/lifted.
    lid.visual(_gasket_mesh(), material=gasket_rubber, name="gasket_ring")

    # Small off-axis marker dot so rotation of the lid is visible.
    lid.visual(
        Cylinder(0.003, 0.002),
        origin=Origin(xyz=(LID_OUTER_R - 0.008, 0.0, LID_TOP_Z - 0.001)),
        material=gasket_rubber,
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
        motion_limits=MotionLimits(lower=0.0, upper=LID_H + 0.01, effort=1.0, velocity=1.0),
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
    # Gasket sits between lid bottom and jar rim — small contact overlap.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="gasket_ring",
        elem_b="jar_glass",
        reason="The gasket ring is seated between the lid underside and the jar rim for sealing.",
    )

    # ---- jar is squat: wider than it is tall ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.01 and bext[1] > bext[2] + 0.01,
        details=f"body extents={bext}",
    )

    # ---- wide mouth: neck/mouth radius is large relative to body ----
    ctx.check(
        "wide mouth (neck radius > 80% of body radius)",
        NECK_R > JAR_OUTER_R * 0.80,
        details=f"neck_r={NECK_R}, body_r={JAR_OUTER_R}, ratio={NECK_R / JAR_OUTER_R:.2f}",
    )

    # ---- thick lid: lid height is substantial ----
    ctx.check(
        "lid is thick (height >= 0.020 m)",
        LID_H >= 0.020,
        details=f"lid_h={LID_H}",
    )

    # ---- gasket ring exists on the lid ----
    gasket_vis = lid.get_visual("gasket_ring")
    ctx.check(
        "gasket ring exists on the lid",
        gasket_vis is not None,
        details="gasket_ring visual not found on lid",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.002,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )
    # Lid caps the neck at rest.
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.03, name="lid caps the wide neck"
    )

    # ---- lid_rotate spins the lid (CONTINUOUS joint) ----
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

    # ---- lid_slide lifts the lid off the jar (PRISMATIC joint) ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H + 0.01}):
        lifted_z = ctx.part_world_position(lid)[2]
        ctx.expect_gap(
            lid, body, axis="z", min_gap=0.0,
            positive_elem="lid_shell", negative_elem="jar_glass",
            name="lifted lid clears the neck",
        )
    ctx.check(
        "lid_slide lifts the lid off the jar",
        lifted_z > rest_z + (LID_H + 0.01) * 0.5,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- gasket moves with the lid when lifted ----
    with ctx.pose({slide: LID_H + 0.01}):
        gasket_lifted = ctx.part_element_world_aabb(lid, elem="gasket_ring")
    gasket_rest = ctx.part_element_world_aabb(lid, elem="gasket_ring")
    gasket_z_rest = (gasket_rest[0][2] + gasket_rest[1][2]) * 0.5
    gasket_z_lift = (gasket_lifted[0][2] + gasket_lifted[1][2]) * 0.5
    ctx.check(
        "gasket moves with the lid when lifted",
        gasket_z_lift > gasket_z_rest + 0.01,
        details=f"gasket rest z={gasket_z_rest}, lifted z={gasket_z_lift}",
    )

    # ---- carrier is massless / has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- at least one non-fixed joint exists ----
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed articulation exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[a.name for a in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
