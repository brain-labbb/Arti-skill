from __future__ import annotations

# Wide APOTHECARY JAR with domed stopper, shaker insert, and gasket ring.
# Variant 25 of the face cream jar family.
#
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
# Wide squat amber glass apothecary jar (~0.09m dia) with a visible wide-mouth
# hollow opening, thick glass walls, a domed screw-on stopper lid, a gasket
# ring seated on the rim, and a shaker insert disc that rotates inside the lid.
#
# Articulations:
#   - lid_rotate: CONTINUOUS spin of carrier about +Z at rim top
#   - lid_slide:  PRISMATIC lift of the lid along +Z (screw-off)
#   - shaker_rotate: CONTINUOUS rotation of shaker insert inside lid about +Z

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
JAR_OUTER_R = 0.045           # outer radius (~0.09m dia, wider apothecary)
JAR_BODY_H = 0.050            # height of the glass body
WALL = 0.005                  # thick glass wall
NECK_R = 0.038                # outer radius of the threaded neck (wide mouth)
NECK_H = 0.012                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim top (0.062)

# Inner cavity: clearly hollow, wide mouth opening
INNER_R = NECK_R - WALL       # inner radius at the mouth (~0.033)
CREAM_TOP_Z = JAR_BODY_H - 0.006  # cream surface just below shoulder

# Gasket dimensions
GASKET_OUTER_R = NECK_R + 0.002   # slightly wider than neck for visible ring
GASKET_INNER_R = INNER_R + 0.001
GASKET_H = 0.003                   # thin rubber gasket

# Lid / stopper dimensions
LID_OUTER_R = NECK_R + 0.004      # lid skirt slightly wider than neck
LID_SKIRT_H = 0.014               # skirt depth
LID_DOME_H = 0.018                # dome height above skirt top
LID_TOTAL_H = LID_SKIRT_H + LID_DOME_H

# Lid part frame origin is at the carrier joint (world z=RIM_TOP_Z).
# Skirt bottom sits below rim to slip over the neck.
LID_SKIRT_BOTTOM_Z = -0.010       # lid-local: 10mm below rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_TOTAL_H  # lid-local dome apex

# Shaker insert
SHAKER_R = INNER_R - 0.002       # fits inside the neck opening
SHAKER_H = 0.003                 # thin disc
SHAKER_N_HOLES = 8               # number of shaker holes
SHAKER_HOLE_R = 0.004            # hole radius


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled wide apothecary jar as a revolved half-profile."""
    pts = [
        (0.0, 0.0),                              # center of base
        (JAR_OUTER_R, 0.0),                      # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.008),       # outer wall up
        (JAR_OUTER_R - 0.005, JAR_BODY_H),       # rounded outer shoulder
        (NECK_R + 0.002, JAR_BODY_H + 0.003),    # step in to neck base
        (NECK_R, JAR_BODY_H + 0.005),            # neck outer wall
        (NECK_R, RIM_TOP_Z),                     # neck up to rim
        (NECK_R - 0.001, RIM_TOP_Z + 0.001),     # small rim lip
        (INNER_R + 0.001, RIM_TOP_Z + 0.001),    # across rim top
        (INNER_R, RIM_TOP_Z),                    # inner rim edge
        (INNER_R, JAR_BODY_H - 0.003),           # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.008),# inner shoulder
        (JAR_OUTER_R - WALL, WALL),              # inner body wall
        (0.0, WALL),                             # inner base
        (0.0, 0.0),                              # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the wide neck for screw-on stopper."""
    threads = None
    z0 = JAR_BODY_H + 0.006
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0008)
            .circle(NECK_R - 0.0004)
            .extrude(0.002)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _cream_surface_mesh():
    """Cream filling inside the jar, visible through the wide mouth."""
    inner_r = JAR_OUTER_R - WALL - 0.001
    disc = (
        cq.Workplane("XY")
        .workplane(offset=CREAM_TOP_Z - 0.010)
        .circle(inner_r)
        .extrude(0.010)
    )
    # Slightly domed top surface like real cream
    dome = (
        cq.Workplane("XY")
        .workplane(offset=CREAM_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.6)
        .loft(ruled=False)
    )
    cream = disc.union(dome)
    return mesh_from_cadquery(cream, "cream_surface")


def _gasket_ring() -> cq.Workplane:
    """Rubber gasket ring that sits on the rim under the lid."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=RIM_TOP_Z - 0.001)
        .circle(GASKET_OUTER_R)
        .circle(GASKET_INNER_R)
        .extrude(GASKET_H)
    )
    return outer


def _lid_solid() -> cq.Workplane:
    """Domed stopper lid: cylindrical skirt + hemisphere-like dome on top."""
    # Skirt: cylindrical cup
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_SKIRT_H)
    )
    # Hollow the skirt interior to slip over the neck
    skirt_cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R + 0.001)
        .extrude(LID_SKIRT_H - 0.003)
    )
    skirt = skirt.cut(skirt_cavity)

    # Dome: lofted from skirt top circle to a smaller circle at dome apex
    skirt_top_z = LID_SKIRT_BOTTOM_Z + LID_SKIRT_H
    dome = (
        cq.Workplane("XY")
        .workplane(offset=skirt_top_z)
        .circle(LID_OUTER_R)
        .workplane(offset=LID_DOME_H * 0.6)
        .circle(LID_OUTER_R * 0.5)
        .workplane(offset=LID_DOME_H * 0.4)
        .circle(LID_OUTER_R * 0.1)
        .loft(ruled=False)
    )
    # Add a small finial knob at the dome apex for grip
    finial = (
        cq.Workplane("XY")
        .workplane(offset=skirt_top_z + LID_DOME_H - 0.002)
        .circle(0.005)
        .extrude(0.005)
    )
    finial = finial.edges(">Z").fillet(0.003)

    lid = skirt.union(dome).union(finial)
    return lid


def _shaker_insert_mesh():
    """Perforated shaker disc that rotates inside the lid.
    The shaker sits just below the lid dome interior, at the top of the skirt."""
    # Base disc
    disc = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z + LID_SKIRT_H - SHAKER_H - 0.001)
        .circle(SHAKER_R)
        .extrude(SHAKER_H)
    )
    # Cut holes in a ring pattern
    holes = None
    hole_circle_r = SHAKER_R * 0.55
    for i in range(SHAKER_N_HOLES):
        ang = 2.0 * math.pi * i / SHAKER_N_HOLES
        hx = hole_circle_r * math.cos(ang)
        hy = hole_circle_r * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=LID_SKIRT_BOTTOM_Z + LID_SKIRT_H - SHAKER_H - 0.002)
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_H + 0.004)
        )
        holes = hole if holes is None else holes.union(hole)
    shaker = disc.cut(holes)
    return mesh_from_cadquery(shaker, "shaker_disc")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apothecary_jar_shaker")

    # Materials
    glass_amber = model.material("glass_amber", rgba=(0.55, 0.35, 0.15, 0.50))
    cream_white = model.material("cream_white", rgba=(0.96, 0.93, 0.88, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.30, 0.30, 0.32, 1.0))
    lid_dark = model.material("lid_dark", rgba=(0.18, 0.14, 0.12, 1.0))
    shaker_silver = model.material("shaker_silver", rgba=(0.72, 0.72, 0.74, 1.0))
    marker_gold = model.material("marker_gold", rgba=(0.78, 0.65, 0.30, 1.0))

    # ---- jar body (root): glass shell + neck threads + cream fill ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(mesh_from_cadquery(glass, "jar_glass"), material=glass_amber, name="jar_glass")

    # Cream visible through wide mouth opening
    body.visual(_cream_surface_mesh(), material=cream_white, name="cream_surface")

    # Gasket ring seated on the rim (fixed to body - it stays on the jar)
    body.visual(
        mesh_from_cadquery(_gasket_ring(), "gasket_ring"),
        material=gasket_gray,
        name="gasket_ring",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- massless carrier (no visuals): rotates about +Z at rim top ----
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

    # ---- lid: domed stopper, slides up off carrier along +Z ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=lid_dark,
        name="lid_shell",
    )
    # Off-axis marker so lid rotation is visible; placed on the dome slope
    # so it contacts the shell surface (slightly proud of the dome).
    lid.visual(
        Box((0.005, 0.005, 0.004)),
        origin=Origin(xyz=(LID_OUTER_R - 0.010, 0.0, LID_SKIRT_BOTTOM_Z + LID_SKIRT_H + 0.006)),
        material=marker_gold,
        name="lid_marker",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_OUTER_R, LID_TOTAL_H),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_TOTAL_H * 0.5)),
    )
    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_TOTAL_H, effort=1.0, velocity=1.0),
    )

    # ---- shaker insert: rotates inside the lid ----
    shaker = model.part("shaker_insert")
    shaker.visual(_shaker_insert_mesh(), material=shaker_silver, name="shaker_disc")
    # Off-axis marker for shaker rotation visibility
    shaker.visual(
        Box((0.003, 0.003, 0.004)),
        origin=Origin(xyz=(SHAKER_R * 0.75, 0.0, LID_SKIRT_BOTTOM_Z + LID_SKIRT_H - SHAKER_H)),
        material=marker_gold,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_SKIRT_H - SHAKER_H * 0.5)),
    )
    model.articulation(
        "shaker_rotate",
        ArticulationType.CONTINUOUS,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
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
    shaker = object_model.get_part("shaker_insert")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    shaker_joint = object_model.get_articulation("shaker_rotate")

    # Allow lid skirt overlapping the neck (seated fit)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim.",
    )

    # ---- jar is wide (wider than tall, apothecary proportions) ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is wide apothecary (wider than tall)",
        bext[0] > bext[2] + 0.005 and bext[1] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- wide-mouth hollow opening: inner cavity radius is large ----
    # The neck inner radius should be at least 60% of the outer body radius
    ctx.check(
        "wide mouth opening (inner radius > 60% of body)",
        INNER_R > JAR_OUTER_R * 0.6,
        details=f"inner_r={INNER_R}, body_r={JAR_OUTER_R}, ratio={INNER_R/JAR_OUTER_R:.2f}",
    )

    # ---- domed stopper exists: lid has a visual named lid_shell ----
    lid_vis_names = [v.name for v in lid.visuals]
    ctx.check(
        "domed stopper lid exists",
        "lid_shell" in lid_vis_names,
        details=f"lid visuals={lid_vis_names}",
    )

    # ---- gasket ring exists on the body ----
    body_vis_names = [v.name for v in body.visuals]
    ctx.check(
        "gasket ring exists under lid",
        "gasket_ring" in body_vis_names,
        details=f"body visuals={body_vis_names}",
    )

    # ---- shaker insert exists and is a separate part ----
    shaker_vis_names = [v.name for v in shaker.visuals]
    ctx.check(
        "shaker insert exists",
        "shaker_disc" in shaker_vis_names,
        details=f"shaker visuals={shaker_vis_names}",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.002,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- lid caps the neck (projected overlap) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.02, name="lid caps the neck"
    )

    # ---- lid_rotate spins the lid (marker moves) ----
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
    with ctx.pose({slide: LID_TOTAL_H}):
        lifted_z = ctx.part_world_position(lid)[2]
        ctx.expect_gap(
            lid, body, axis="z", min_gap=0.0,
            positive_elem="lid_shell", negative_elem="jar_glass",
            name="lifted lid clears the neck",
        )
    ctx.check(
        "lid_slide lifts the lid off the jar",
        lifted_z > rest_z + LID_TOTAL_H * 0.4,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- shaker_rotate spins the shaker independently ----
    shaker_marker0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    sm0 = ((shaker_marker0[0][0] + shaker_marker0[1][0]) * 0.5,
           (shaker_marker0[0][1] + shaker_marker0[1][1]) * 0.5)
    with ctx.pose({shaker_joint: math.pi}):
        shaker_marker1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        sm1 = ((shaker_marker1[0][0] + shaker_marker1[1][0]) * 0.5,
               (shaker_marker1[0][1] + shaker_marker1[1][1]) * 0.5)
    shaker_moved = math.hypot(sm1[0] - sm0[0], sm1[1] - sm0[1])
    ctx.check(
        "shaker_rotate spins the shaker insert (marker moves)",
        shaker_moved > 0.005,
        details=f"shaker rest={sm0}, half-turn={sm1}, moved={shaker_moved}",
    )

    # ---- carrier has no visuals ----
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
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[a.name for a in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
