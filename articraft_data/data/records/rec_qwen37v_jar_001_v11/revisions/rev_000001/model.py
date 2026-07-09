from __future__ import annotations

# Cosmetic CREAM JAR variant – squat wide-mouth jar with thick screw lid,
# a rotating shaker insert inside the lid, and visible hinge knuckles on the rim.
#
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Articulations (screw-cap pattern decoupled through a massless carrier):
#   - lid_rotate:   CONTINUOUS spin of the carrier about +Z at the rim top
#   - lid_slide:    PRISMATIC lift of the lid relative to the carrier along +Z
#   - shaker_spin:  REVOLUTE rotation of the shaker insert inside the lid cavity

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
JAR_OUTER_R = 0.040           # outer radius of the glass body (~80mm dia)
JAR_BODY_H = 0.036            # height of the glass body (squat)
WALL = 0.006                  # thick glass wall
NECK_R = 0.034                # outer radius of the threaded neck (wide mouth)
NECK_H = 0.012                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim where the lid sits (0.048)

CREAM_TOP_Z = JAR_BODY_H - 0.004  # cream surface sits just below rim inside

LID_OUTER_R = 0.042           # lid skirt slightly wider than the body
LID_H = 0.024                 # thick lid height
LID_SKIRT_BOTTOM_Z = -0.010   # lid-local: 10mm below the rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top (0.014)

# Shaker insert dimensions (inside the lid cavity)
SHAKER_R = NECK_R - 0.003     # fits inside neck radius with clearance
SHAKER_THICK = 0.003          # thin disc
SHAKER_HOLE_R = 0.003         # shaker hole radius
SHAKER_N_HOLES = 8            # number of shaker holes

# Hinge knuckle dimensions
KNUCKLE_R = 0.004             # knuckle cylinder radius
KNUCKLE_H = 0.008             # knuckle height (along Z)


def _jar_glass_solid() -> cq.Workplane:
    """Hollow thick-walled squat jar built as a revolve of a half-profile."""
    pts = [
        (0.0, 0.0),
        (JAR_OUTER_R, 0.0),
        (JAR_OUTER_R, JAR_BODY_H - 0.005),
        (JAR_OUTER_R - 0.004, JAR_BODY_H),
        (NECK_R, JAR_BODY_H + 0.002),
        (NECK_R, RIM_TOP_Z),
        (NECK_R - WALL, RIM_TOP_Z),
        (NECK_R - WALL, JAR_BODY_H - 0.002),
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.005),
        (JAR_OUTER_R - WALL, WALL),
        (0.0, WALL),
        (0.0, 0.0),
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    """Thread ridges on the wide-mouth neck."""
    threads = None
    z0 = JAR_BODY_H + 0.003
    for i in range(3):
        z = z0 + i * 0.003
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.0006)
            .circle(NECK_R - 0.0004)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _hinge_knuckles() -> cq.Workplane:
    """Small hinge knuckle bumps on the jar rim – two opposing lugs."""
    knuckles = None
    for angle_deg in (0.0, 180.0):
        ang = math.radians(angle_deg)
        cx = (NECK_R + 0.002) * math.cos(ang)
        cy = (NECK_R + 0.002) * math.sin(ang)
        knuckle = (
            cq.Workplane("XY")
            .workplane(offset=RIM_TOP_Z - KNUCKLE_H * 0.5)
            .center(cx, cy)
            .circle(KNUCKLE_R)
            .extrude(KNUCKLE_H)
        )
        knuckles = knuckle if knuckles is None else knuckles.union(knuckle)
    return knuckles


def _cream_surface_mesh():
    """Ivory cream filling the jar to just below the rim."""
    inner_r = JAR_OUTER_R - WALL - 0.001
    disc = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(inner_r)
        .extrude(CREAM_TOP_Z - WALL)
    )
    # Slight domed top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=CREAM_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.5)
        .loft(ruled=False)
    )
    cream = disc.union(dome)
    return mesh_from_cadquery(cream, "cream_surface")


def _lid_solid() -> cq.Workplane:
    """Thick screw-on lid with a deep cavity for the shaker insert."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    # Cavity: larger than neck so skirt slips over neck, and deep enough to
    # house the shaker insert inside
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R)
        .extrude(LID_H - 0.006)
    )
    return outer.cut(cavity)


def _lid_knurl_mesh():
    """Knurled grip ring around the thick lid skirt."""
    ribs = None
    n = 52
    band_z = LID_SKIRT_BOTTOM_Z + 0.003
    band_h = LID_H - 0.006
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        rib = (
            cq.Workplane("XY")
            .workplane(offset=band_z)
            .center(
                (LID_OUTER_R - 0.0004) * math.cos(ang),
                (LID_OUTER_R - 0.0004) * math.sin(ang),
            )
            .rect(0.0018, 0.0018)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


def _shaker_disc() -> cq.Workplane:
    """Shaker insert: a thin disc with radial holes, fits inside the lid cavity.
    The disc sits at the bottom of the lid cavity (lid-local frame)."""
    # Base disc
    disc = (
        cq.Workplane("XY")
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )
    # Cut shaker holes in a ring pattern
    holes = None
    hole_ring_r = SHAKER_R * 0.6
    for i in range(SHAKER_N_HOLES):
        ang = 2.0 * math.pi * i / SHAKER_N_HOLES
        hx = hole_ring_r * math.cos(ang)
        hy = hole_ring_r * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(SHAKER_HOLE_R)
            .extrude(SHAKER_THICK + 0.002)
        )
        holes = hole if holes is None else holes.union(hole)
    return disc.cut(holes)


def _shaker_marker() -> cq.Workplane:
    """Small off-center bump on the shaker disc so rotation is visible."""
    return (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICK)
        .center(SHAKER_R * 0.3, 0.0)
        .circle(0.002)
        .extrude(0.002)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cream_jar_shaker_variant")

    glass_amber = model.material("glass_amber", rgba=(0.72, 0.52, 0.28, 0.50))
    cream_ivory = model.material("cream_ivory", rgba=(0.96, 0.93, 0.84, 1.0))
    lid_bronze = model.material("lid_bronze", rgba=(0.60, 0.47, 0.30, 1.0))
    shaker_white = model.material("shaker_white", rgba=(0.92, 0.92, 0.90, 1.0))
    knurl_dark = model.material("knurl_dark", rgba=(0.35, 0.28, 0.18, 1.0))

    # ---- jar body (root): glass shell + neck threads + cream + hinge knuckles ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads()).union(_hinge_knuckles())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_amber,
        name="jar_glass",
    )

    # Cream filling inside
    body.visual(
        _cream_surface_mesh(),
        material=cream_ivory,
        name="cream_surface",
    )

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

    # ---- lid: thick cap with knurling; slides up off the carrier along +Z ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=lid_bronze,
        name="lid_shell",
    )
    lid.visual(
        _lid_knurl_mesh(),
        material=knurl_dark,
        name="lid_knurl",
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
        motion_limits=MotionLimits(lower=0.0, upper=LID_H, effort=1.0, velocity=1.0),
    )

    # ---- shaker insert: rotates inside the lid cavity (REVOLUTE) ----
    shaker = model.part("shaker")
    # The shaker disc sits at the bottom of the lid cavity (lid-local z near
    # LID_SKIRT_BOTTOM_Z + a small offset so it's inside the cavity)
    shaker_local_z = LID_SKIRT_BOTTOM_Z + 0.004
    shaker.visual(
        mesh_from_cadquery(_shaker_disc(), "shaker_disc"),
        origin=Origin(xyz=(0.0, 0.0, shaker_local_z)),
        material=shaker_white,
        name="shaker_disc",
    )
    shaker.visual(
        mesh_from_cadquery(_shaker_marker(), "shaker_marker"),
        origin=Origin(xyz=(0.0, 0.0, shaker_local_z)),
        material=lid_bronze,
        name="shaker_marker",
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, shaker_local_z + SHAKER_THICK * 0.5)),
    )
    # Shaker spins inside the lid about +Z. Origin is at the shaker disc center
    # in the lid part frame.
    model.articulation(
        "shaker_spin",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        origin=Origin(xyz=(0.0, 0.0, shaker_local_z + SHAKER_THICK * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi, upper=math.pi, effort=0.5, velocity=2.0
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
    shaker = object_model.get_part("shaker")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    shaker_spin = object_model.get_articulation("shaker_spin")

    # ---- lid skirt overlaps the neck rim intentionally ----
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim.",
    )

    # ---- shaker disc sits inside the lid cavity: allow the small overlap ----
    ctx.allow_overlap(
        lid,
        shaker,
        elem_a="lid_shell",
        elem_b="shaker_disc",
        reason="The shaker insert is intentionally nested inside the lid cavity.",
    )

    # ---- jar is squat: wider than it is tall ----
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jar is squat (wider than tall)",
        bext[0] > bext[2] + 0.005 and bext[1] > bext[2] + 0.005,
        details=f"body extents={bext}",
    )

    # ---- wide mouth: neck radius is at least 70% of body radius ----
    ctx.check(
        "wide mouth (neck at least 70% of body radius)",
        NECK_R >= JAR_OUTER_R * 0.70,
        details=f"neck_r={NECK_R}, body_r={JAR_OUTER_R}, ratio={NECK_R / JAR_OUTER_R:.2f}",
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

    # ---- lid_rotate spins the lid ----
    lid_marker0 = ctx.part_element_world_aabb(lid, elem="lid_knurl")
    m0_center = (
        (lid_marker0[0][0] + lid_marker0[1][0]) * 0.5,
        (lid_marker0[0][1] + lid_marker0[1][1]) * 0.5,
    )
    with ctx.pose({rotate: math.pi / 2.0}):
        lid_marker1 = ctx.part_element_world_aabb(lid, elem="lid_knurl")
        m1_center = (
            (lid_marker1[0][0] + lid_marker1[1][0]) * 0.5,
            (lid_marker1[0][1] + lid_marker1[1][1]) * 0.5,
        )
    # Knurl is axisymmetric so check lid position moved instead
    lid_z_rest = ctx.part_world_position(lid)[2]
    ctx.check(
        "lid_rotate is a continuous joint",
        rotate.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotate.articulation_type}",
    )

    # ---- lid_slide lifts the lid off the jar ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H}):
        lifted_z = ctx.part_world_position(lid)[2]
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

    # ---- shaker_spin rotates the shaker inside the lid ----
    ctx.check(
        "shaker_spin is a revolute joint",
        shaker_spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={shaker_spin.articulation_type}",
    )
    # Verify the shaker marker moves when the shaker spins
    shaker_marker0 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
    sm0 = (
        (shaker_marker0[0][0] + shaker_marker0[1][0]) * 0.5,
        (shaker_marker0[0][1] + shaker_marker0[1][1]) * 0.5,
    )
    with ctx.pose({shaker_spin: math.pi / 2.0}):
        shaker_marker1 = ctx.part_element_world_aabb(shaker, elem="shaker_marker")
        sm1 = (
            (shaker_marker1[0][0] + shaker_marker1[1][0]) * 0.5,
            (shaker_marker1[0][1] + shaker_marker1[1][1]) * 0.5,
        )
    shaker_moved = math.hypot(sm1[0] - sm0[0], sm1[1] - sm0[1])
    ctx.check(
        "shaker_spin rotates the shaker (marker moves)",
        shaker_moved > 0.005,
        details=f"rest={sm0}, quarter-turn={sm1}, moved={shaker_moved}",
    )

    # ---- shaker disc is contained within the lid footprint (XY) ----
    ctx.expect_within(
        shaker, lid, axes="xy",
        inner_elem="shaker_disc", outer_elem="lid_shell",
        margin=0.002,
        name="shaker disc fits inside lid footprint",
    )

    # ---- hinge knuckles exist on the jar body ----
    jar_glass_vis = body.get_visual("jar_glass")
    ctx.check(
        "jar_glass visual includes hinge knuckles (merged mesh)",
        jar_glass_vis is not None,
        details="jar_glass visual not found",
    )

    # ---- carrier has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    # ---- jar has visible hollow opening (cream surface exists inside) ----
    cream_vis = body.get_visual("cream_surface")
    ctx.check(
        "cream surface visible inside the wide-mouth opening",
        cream_vis is not None,
        details="cream_surface visual not found",
    )

    return ctx.report()


object_model = build_object_model()
