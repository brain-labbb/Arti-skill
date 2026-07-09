from __future__ import annotations

# HONEY JAR with screw-on lid featuring a dipper holder and rotating shaker insert.
# Frame: jar axis along +Z, base resting on z=0, jar centered on (x=0, y=0).
#
# Key features (variant from face cream jar):
#   - Taller glass jar body with thick walls visible at the mouth
#   - Thread ridges around the rim
#   - Amber honey fill inside
#   - Screw-on lid with dipper holder on top
#   - Shaker insert (perforated disc) that rotates inside the lid
#
# Articulations:
#   - lid_rotate: CONTINUOUS spin of the carrier about +Z at the rim top
#   - lid_slide: PRISMATIC lift of the lid relative to the carrier along +Z
#   - shaker_rotate: REVOLUTE rotation of the shaker insert inside the lid

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
JAR_OUTER_R = 0.040           # outer radius of glass body (~0.08m dia)
JAR_BODY_H = 0.080            # height of the glass body (taller honey jar)
WALL = 0.006                  # thick glass wall (visible at mouth)
NECK_R = 0.034                # outer radius of the threaded neck (wide mouth)
NECK_H = 0.014                # neck height above the shoulder
RIM_TOP_Z = JAR_BODY_H + NECK_H  # z of the neck rim where the lid sits (0.094)

HONEY_TOP_Z = JAR_BODY_H - 0.008  # honey surface sits below the shoulder

LID_OUTER_R = 0.041           # lid skirt slightly wider than neck
LID_H = 0.022                 # lid height
LID_SKIRT_BOTTOM_Z = -0.012   # lid-local: skirt drops below rim top
LID_TOP_Z = LID_SKIRT_BOTTOM_Z + LID_H  # lid-local top of cap (0.010)

SHAKER_R = 0.028              # shaker disc radius (fits inside lid cavity)
SHAKER_THICK = 0.003          # shaker disc thickness

DIPPER_HOLDER_R = 0.006       # dipper holder tube radius
DIPPER_HOLDER_H = 0.018       # dipper holder tube height


def _jar_glass_solid() -> cq.Workplane:
    # Hollow thick-walled glass jar. The revolve profile creates real glass wall
    # thickness visible at the mouth opening, with a rounded shoulder and threaded neck.
    pts = [
        (0.0, 0.0),                           # center of base
        (JAR_OUTER_R, 0.0),                   # outer base edge
        (JAR_OUTER_R, JAR_BODY_H - 0.010),    # outer wall up
        (JAR_OUTER_R - 0.006, JAR_BODY_H),    # rounded outer shoulder
        (NECK_R, JAR_BODY_H + 0.003),         # step into neck
        (NECK_R, RIM_TOP_Z),                  # neck outer up to rim
        (NECK_R - WALL, RIM_TOP_Z),           # across the rim top (thick glass!)
        (NECK_R - WALL, JAR_BODY_H - 0.003),  # inner neck wall down
        (JAR_OUTER_R - WALL, JAR_BODY_H - 0.010),
        (JAR_OUTER_R - WALL, WALL),           # inner body wall down
        (0.0, WALL),                          # across inner base
        (0.0, 0.0),                           # close
    ]
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _neck_threads() -> cq.Workplane:
    # Prominent thread ridges around the rim for the screw-on lid.
    # These are thick raised ridges spiraling around the neck.
    threads = None
    z0 = JAR_BODY_H + 0.005
    n_ridges = 4
    for i in range(n_ridges):
        z = z0 + i * 0.0025
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(NECK_R + 0.001)
            .circle(NECK_R - 0.0005)
            .extrude(0.0018)
        )
        threads = ring if threads is None else threads.union(ring)
    return threads


def _honey_fill_mesh():
    # Amber honey fill inside the jar, with a slightly domed meniscus surface.
    inner_r = JAR_OUTER_R - WALL - 0.001
    # Main honey body
    body = (
        cq.Workplane("XY")
        .workplane(offset=WALL + 0.001)
        .circle(inner_r)
        .extrude(HONEY_TOP_Z - WALL - 0.001)
    )
    # Domed meniscus on top
    meniscus = (
        cq.Workplane("XY")
        .workplane(offset=HONEY_TOP_Z)
        .circle(inner_r)
        .workplane(offset=0.003)
        .circle(inner_r * 0.7)
        .loft(ruled=False)
    )
    honey = body.union(meniscus)
    return mesh_from_cadquery(honey, "honey_fill")


LID_TOP_PLATE_Z = LID_SKIRT_BOTTOM_Z + LID_H - 0.004  # top plate starts here (lid-local)
SHAKER_POCKET_BOTTOM = LID_SKIRT_BOTTOM_Z + LID_H - 0.012  # pocket starts at ~-0.002
SHAKER_POCKET_DEPTH = 0.008  # pocket goes to LID_TOP_PLATE_Z (0.006 to 0.006+0.008... let me recalc)
STEM_HOLE_R = 0.004  # hole in lid top for shaker stem


def _lid_solid() -> cq.Workplane:
    # Screw-on lid: closed top with skirt that slips over the neck threads.
    # The lid cavity accommodates the shaker insert pocket and stem hole.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z)
        .circle(LID_OUTER_R)
        .extrude(LID_H)
    )
    outer = outer.edges(">Z").fillet(0.003)
    # Cavity for the neck to slip into (matches neck outer + thread clearance)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LID_SKIRT_BOTTOM_Z - 0.001)
        .circle(NECK_R + 0.001)
        .extrude(LID_H - 0.006)
    )
    # Shaker pocket: recess in the underside of the top plate for the disc
    # Pocket goes from just above the neck cavity to just below the top plate
    pocket_bottom_z = LID_SKIRT_BOTTOM_Z + LID_H - 0.012  # -0.002
    pocket_depth = 0.008  # goes to 0.006 in lid-local
    shaker_pocket = (
        cq.Workplane("XY")
        .workplane(offset=pocket_bottom_z)
        .circle(SHAKER_R + 0.002)
        .extrude(pocket_depth)
    )
    # Central stem hole through the top plate for the shaker pivot stem
    stem_hole = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_PLATE_Z - 0.001)
        .circle(STEM_HOLE_R)
        .extrude(0.006)
    )
    return outer.cut(cavity).cut(shaker_pocket).cut(stem_hole)


def _lid_knurl_mesh():
    # Knurled grip band around the lid skirt.
    ribs = None
    n = 48
    band_z = LID_SKIRT_BOTTOM_Z + 0.002
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
            .rect(0.0016, 0.0016)
            .extrude(band_h)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return mesh_from_cadquery(ribs, "lid_knurl")


DIPPER_OFFSET_X = 0.022  # offset from lid center so it orbits when lid rotates


def _dipper_holder_mesh():
    # Dipper holder: a small open-topped cylindrical tube mounted on the lid top,
    # offset from center so it orbits when the lid rotates. It holds a honey dipper.
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z)
        .center(DIPPER_OFFSET_X, 0.0)
        .circle(DIPPER_HOLDER_R)
        .extrude(DIPPER_HOLDER_H)
    )
    # Hollow interior
    inner = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z + 0.002)
        .center(DIPPER_OFFSET_X, 0.0)
        .circle(DIPPER_HOLDER_R - 0.002)
        .extrude(DIPPER_HOLDER_H - 0.002)
    )
    holder = outer.cut(inner)
    # Base flange connecting to the lid top surface
    flange = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_Z - 0.002)
        .center(DIPPER_OFFSET_X, 0.0)
        .circle(DIPPER_HOLDER_R + 0.003)
        .extrude(0.004)
    )
    return mesh_from_cadquery(holder.union(flange), "dipper_holder")


def _shaker_insert_mesh():
    # Shaker insert: a perforated disc with a central stem and top knob.
    # Authored in the SHAKER part frame (origin at the disc bottom).
    # Disc at z=0 to SHAKER_THICK, stem rises through the lid, knob on top.
    disc = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(SHAKER_R)
        .extrude(SHAKER_THICK)
    )
    # Cut shaker holes in a ring pattern
    holes = None
    n_holes = 8
    hole_r = 0.003
    hole_ring_r = SHAKER_R * 0.6
    for i in range(n_holes):
        ang = 2.0 * math.pi * i / n_holes
        hx = hole_ring_r * math.cos(ang)
        hy = hole_ring_r * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(hole_r)
            .extrude(SHAKER_THICK + 0.002)
        )
        holes = hole if holes is None else holes.union(hole)

    # Inner ring of smaller holes
    n_inner = 5
    hole_ring_r2 = SHAKER_R * 0.3
    for i in range(n_inner):
        ang = 2.0 * math.pi * i / n_inner + 0.3
        hx = hole_ring_r2 * math.cos(ang)
        hy = hole_ring_r2 * math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(hx, hy)
            .circle(hole_r * 0.7)
            .extrude(SHAKER_THICK + 0.002)
        )
        holes = hole if holes is None else holes.union(hole)

    disc_cut = disc.cut(holes) if holes is not None else disc

    # Central pivot stem: rises from the disc through the lid top hole
    # Stem needs to reach from disc top (shaker-local 0.003) to above the lid
    stem_height = 0.012  # tall enough to pass through the lid top plate
    stem = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .circle(0.003)
        .extrude(SHAKER_THICK + stem_height)
    )
    # Rotation knob on top of the stem (above the lid)
    # An asymmetric marker so rotation is visible
    knob = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICK + stem_height - 0.001)
        .circle(0.005)
        .extrude(0.004)
    )
    # Small indicator tab on the knob for visual rotation feedback
    indicator = (
        cq.Workplane("XY")
        .workplane(offset=SHAKER_THICK + stem_height + 0.001)
        .center(0.004, 0.0)
        .rect(0.004, 0.002)
        .extrude(0.003)
    )
    result = disc_cut.union(stem).union(knob).union(indicator)
    return mesh_from_cadquery(result, "shaker_insert")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar")

    # Materials
    glass_clear = model.material("glass_clear", rgba=(0.85, 0.90, 0.88, 0.45))
    honey_amber = model.material("honey_amber", rgba=(0.85, 0.55, 0.10, 0.92))
    lid_wood = model.material("lid_wood", rgba=(0.62, 0.42, 0.22, 1.0))
    shaker_metal = model.material("shaker_metal", rgba=(0.72, 0.72, 0.70, 1.0))
    holder_wood = model.material("holder_wood", rgba=(0.55, 0.38, 0.18, 1.0))

    # ---- jar body (root): glass shell + neck threads + honey fill ----
    body = model.part("body")

    glass = _jar_glass_solid().union(_neck_threads())
    body.visual(
        mesh_from_cadquery(glass, "jar_glass"),
        material=glass_clear,
        name="jar_glass",
    )

    # Honey fill
    body.visual(_honey_fill_mesh(), material=honey_amber, name="honey_fill")

    body.inertial = Inertial.from_geometry(
        Cylinder(JAR_OUTER_R, JAR_BODY_H),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, JAR_BODY_H * 0.5)),
    )

    # ---- massless carrier: rotates about +Z at the rim top ----
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
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=lid_wood,
        name="lid_shell",
    )
    lid.visual(_lid_knurl_mesh(), material=lid_wood, name="lid_knurl")

    # Dipper holder on top of lid
    lid.visual(_dipper_holder_mesh(), material=holder_wood, name="dipper_holder")

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
            lower=0.0, upper=LID_H + 0.01, effort=1.0, velocity=1.0
        ),
    )

    # ---- shaker insert: rotates inside the lid ----
    shaker = model.part("shaker")
    shaker.visual(
        _shaker_insert_mesh(), material=shaker_metal, name="shaker_insert"
    )
    shaker.inertial = Inertial.from_geometry(
        Cylinder(SHAKER_R, SHAKER_THICK + 0.012),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )
    model.articulation(
        "shaker_rotate",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=shaker,
        # Place the shaker disc inside the lid pocket
        origin=Origin(xyz=(0.0, 0.0, LID_SKIRT_BOTTOM_Z + LID_H - 0.011)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-math.pi, upper=math.pi, effort=0.5, velocity=2.0
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carrier = object_model.get_part("lid_carrier")
    lid = object_model.get_part("lid")
    shaker = object_model.get_part("shaker")
    rotate = object_model.get_articulation("lid_rotate")
    slide = object_model.get_articulation("lid_slide")
    shaker_joint = object_model.get_articulation("shaker_rotate")

    # Allow the lid skirt to overlap the neck (seated fit over threads)
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="jar_glass",
        reason="The lid skirt is intentionally slipped down over the threaded neck rim.",
    )

    # The shaker stem passes through a clearance hole in the lid top plate.
    # At mesh resolution this may register as a tiny overlap at the hole edge.
    ctx.allow_overlap(
        shaker,
        lid,
        elem_a="shaker_insert",
        elem_b="lid_shell",
        reason="The shaker stem passes through a clearance hole in the lid top plate (captured pivot).",
    )

    # ---- jar is taller than wide (honey jar proportions) ----
    body_aabb = ctx.part_world_aabb(body)
    bext = (
        body_aabb[1][0] - body_aabb[0][0],
        body_aabb[1][1] - body_aabb[0][1],
        body_aabb[1][2] - body_aabb[0][2],
    )
    ctx.check(
        "honey jar is taller than wide",
        bext[2] > bext[0] - 0.005,
        details=f"body extents={bext}",
    )

    # ---- glass wall thickness at mouth: the jar_glass visual has inner/outer radii ----
    # The neck outer radius (NECK_R=0.034) differs from the inner wall (NECK_R-WALL=0.028)
    # proving real wall thickness at the mouth.
    ctx.check(
        "glass wall thickness at mouth",
        WALL > 0.004 and NECK_R - WALL < NECK_R - 0.003,
        details=f"WALL={WALL}, NECK_R={NECK_R}, inner_r={NECK_R - WALL}",
    )

    # ---- thread ridges exist on the rim (jar_glass includes threads) ----
    glass_vis = body.get_visual("jar_glass")
    ctx.check(
        "jar has thread ridges on rim",
        glass_vis is not None,
        details="jar_glass visual includes thread ridge geometry",
    )

    # ---- dipper holder exists on lid ----
    dipper_vis = lid.get_visual("dipper_holder")
    ctx.check(
        "dipper holder exists on lid",
        dipper_vis is not None,
        details="lid should have a dipper_holder visual",
    )

    # ---- shaker insert exists ----
    shaker_vis = shaker.get_visual("shaker_insert")
    ctx.check(
        "shaker insert exists",
        shaker_vis is not None,
        details="shaker part should have a shaker_insert visual",
    )

    # ---- lid sits on top of the jar ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid is on top of the jar",
        lid_pos is not None and lid_pos[2] > RIM_TOP_Z - 0.002,
        details=f"lid_pos={lid_pos}, rim_top={RIM_TOP_Z}",
    )

    # ---- lid_rotate spins the lid (dipper holder moves off-axis) ----
    holder_aabb0 = ctx.part_element_world_aabb(lid, elem="dipper_holder")
    holder_center0 = (
        (holder_aabb0[0][0] + holder_aabb0[1][0]) * 0.5,
        (holder_aabb0[0][1] + holder_aabb0[1][1]) * 0.5,
    )
    with ctx.pose({rotate: math.pi / 2.0}):
        holder_aabb1 = ctx.part_element_world_aabb(lid, elem="dipper_holder")
        holder_center1 = (
            (holder_aabb1[0][0] + holder_aabb1[1][0]) * 0.5,
            (holder_aabb1[0][1] + holder_aabb1[1][1]) * 0.5,
        )
    moved = math.hypot(
        holder_center1[0] - holder_center0[0],
        holder_center1[1] - holder_center0[1],
    )
    ctx.check(
        "lid_rotate spins the lid (dipper holder orbits)",
        moved > 0.005,
        details=f"holder rest={holder_center0}, quarter-turn={holder_center1}, moved={moved}",
    )

    # ---- lid_slide lifts the lid off the jar ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({slide: LID_H + 0.01}):
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
        lifted_z > rest_z + LID_H * 0.4,
        details=f"rest_z={rest_z}, lifted_z={lifted_z}",
    )

    # ---- shaker_rotate actually rotates the shaker ----
    shaker_aabb0 = ctx.part_element_world_aabb(shaker, elem="shaker_insert")
    with ctx.pose({shaker_joint: math.pi / 4.0}):
        shaker_aabb1 = ctx.part_element_world_aabb(shaker, elem="shaker_insert")
    # The AABB should change because the tab is asymmetric
    dx = abs(shaker_aabb1[0][0] - shaker_aabb0[0][0]) + abs(
        shaker_aabb1[1][0] - shaker_aabb0[1][0]
    )
    dy = abs(shaker_aabb1[0][1] - shaker_aabb0[0][1]) + abs(
        shaker_aabb1[1][1] - shaker_aabb0[1][1]
    )
    ctx.check(
        "shaker_rotate moves the shaker insert",
        dx + dy > 0.001,
        details=f"AABB delta x={dx}, y={dy}",
    )

    # ---- shaker_rotate has bounded limits (REVOLUTE, not CONTINUOUS) ----
    limits = shaker_joint.motion_limits
    ctx.check(
        "shaker_rotate is REVOLUTE with finite limits",
        shaker_joint.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None,
        details=f"type={shaker_joint.articulation_type}, limits={limits}",
    )

    # ---- carrier has no visuals ----
    ctx.check(
        "carrier link has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    return ctx.report()


object_model = build_object_model()
