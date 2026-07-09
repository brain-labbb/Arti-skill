from __future__ import annotations

# Hexagonal brushed-metal tea tin with a lift-off lid.
# Frame: hexagonal prism axis along +Z, centered on the world origin in XY.
#   - body: a tall hollow hexagonal-prism tin sitting on the ground at z=0.
#   - lid: a matching hexagonal cap with a short overhanging skirt that slips
#     down over the body rim. It lifts STRAIGHT UP (+Z) off the tin (prismatic),
#     with no thread or hinge.
#
# Scale: ~0.07 m across flats, body ~0.13 m tall. The lid skirt overlaps the
# body's top rim when seated (down), and the joint travels up ~0.04 m.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Hexagon sizing (flat-to-flat "across flats" dimension) ---
ACROSS_FLATS = 0.070  # body across-flats width
# polygon(6, diameter) inscribes vertices on a circle of radius diameter/2.
# across_flats = circumradius * 2 * cos(30deg) -> circumradius = af / (2*cos30)
BODY_CIRCUM = ACROSS_FLATS / (2.0 * math.cos(math.radians(30.0)))
BODY_DIAM = BODY_CIRCUM * 2.0  # diameter arg for cq.polygon

WALL = 0.0025  # metal wall thickness
BODY_INNER_CIRCUM = BODY_CIRCUM - WALL / math.cos(math.radians(30.0))
BODY_INNER_DIAM = BODY_INNER_CIRCUM * 2.0

BODY_HEIGHT = 0.130  # tall tin body
BODY_FLOOR = 0.004  # floor thickness at the bottom

# Lid: slightly larger across-flats so its skirt overhangs the body rim.
LID_CLEAR = 0.0012  # radial clearance so the skirt slips over the rim
LID_CIRCUM = BODY_CIRCUM + WALL + LID_CLEAR
LID_DIAM = LID_CIRCUM * 2.0
LID_INNER_CIRCUM = BODY_CIRCUM + LID_CLEAR  # inner of skirt rides just outside body wall
LID_INNER_DIAM = LID_INNER_CIRCUM * 2.0

LID_TOP_THICK = 0.012  # solid top cap thickness
LID_SKIRT = 0.018  # depth the skirt drapes down over the body
SKIRT_OVERLAP = 0.010  # how far the skirt overlaps the body rim when seated

LIFT_TRAVEL = 0.040  # lid lifts straight up this far


def _hex_body() -> cq.Workplane:
    # Hollow hexagonal prism: outer extrude minus an inner pocket, leaving a
    # closed floor and an open top. Built standing along +Z, base at z=0.
    outer = (
        cq.Workplane("XY")
        .polygon(6, BODY_DIAM)
        .extrude(BODY_HEIGHT)
    )
    # Inner cavity starts above the floor and opens at the top.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_FLOOR)
        .polygon(6, BODY_INNER_DIAM)
        .extrude(BODY_HEIGHT)  # extends past the top to fully open it
    )
    return outer.cut(cavity)


def _hex_lid() -> cq.Workplane:
    # Hexagonal cap: a solid top plate with a thin skirt hanging down around the
    # rim. Built so the joint origin (lid frame origin) sits at the seam plane
    # where the lid meets the body rim. Local +Z is the top of the lid; the
    # skirt drops below into -Z so it overlaps the body rim when seated.
    #
    # Top plate sits above the seam, skirt hangs below the seam.
    top = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .polygon(6, LID_DIAM)
        .extrude(LID_TOP_THICK)
    )
    # Skirt: hollow hex ring hanging down from the seam.
    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT)
        .polygon(6, LID_DIAM)
        .extrude(LID_SKIRT)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT - 0.001)
        .polygon(6, LID_INNER_DIAM)
        .extrude(LID_SKIRT + 0.002)
    )
    skirt = skirt_outer.cut(skirt_inner)
    return top.union(skirt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexagonal_tea_tin")

    brushed_metal = model.material("brushed_metal", rgba=(0.74, 0.76, 0.79, 1.0))

    # ---- body (root): hollow hexagonal-prism tin standing on the ground ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_hex_body(), "tin_body"),
        material=brushed_metal,
        name="tin_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((ACROSS_FLATS, ACROSS_FLATS, BODY_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    # ---- lid: hexagonal cap with overhanging skirt, lifts straight up ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_hex_lid(), "tin_lid"),
        material=brushed_metal,
        name="tin_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DIAM, LID_DIAM, LID_TOP_THICK + LID_SKIRT)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, (LID_TOP_THICK - LID_SKIRT) / 2.0)),
    )

    # Seam plane: the lid seats so its skirt overlaps the body rim. The joint
    # origin is at the seam height (body top minus the seated overlap), so when
    # down the skirt (-Z of lid) overlaps the rim, and positive prismatic travel
    # lifts the lid straight up along +Z.
    seam_z = BODY_HEIGHT - SKIRT_OVERLAP
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, seam_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=10.0,
            velocity=0.2,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("body_to_lid")

    # Body is a hexagonal prism: taller than it is wide (across flats).
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is taller than wide (tall hex prism)",
        body_ext[2] > max(body_ext[0], body_ext[1]) + 0.03,
        details=f"body extents={body_ext}",
    )
    # Across-flats footprint is roughly the intended ~0.07 m.
    ctx.check(
        "body across-flats footprint near 0.07 m",
        0.06 < max(body_ext[0], body_ext[1]) < 0.085,
        details=f"body extents={body_ext}",
    )
    # Hexagonal (6-sided) cross-section: the two horizontal extents differ
    # (flat-to-flat vs vertex-to-vertex), unlike a square or circle.
    ctx.check(
        "cross-section is hexagonal, not square/round",
        abs(body_ext[0] - body_ext[1]) > 0.003,
        details=f"x={body_ext[0]:.4f}, y={body_ext[1]:.4f}",
    )

    # Lid skirt overhangs the body rim: lid footprint is slightly larger.
    lid_ext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid overhangs the body rim",
        max(lid_ext[0], lid_ext[1]) > max(body_ext[0], body_ext[1]) + 0.0005,
        details=f"lid={lid_ext}, body={body_ext}",
    )

    # When seated (down), the lid skirt overlaps the body rim region. The skirt
    # intentionally slips over the outside of the body wall.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="tin_lid",
        elem_b="tin_body",
        reason="The lid skirt is intentionally seated over the outside of the body rim.",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="z",
        min_overlap=0.004,
        name="lid skirt overlaps body rim when seated",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.04,
        name="lid sits concentric over the body footprint",
    )

    # The lid lifts straight up off the tin (prismatic +Z), no rotation.
    rest = ctx.part_world_position(lid)
    with ctx.pose({lift: LIFT_TRAVEL}):
        lifted = ctx.part_world_position(lid)
        lifted_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid lifts straight up off the tin",
        lifted[2] > rest[2] + 0.035
        and abs(lifted[0] - rest[0]) < 1e-6
        and abs(lifted[1] - rest[1]) < 1e-6,
        details=f"rest={rest}, lifted={lifted}",
    )
    # Fully lifted, the lid clears the body top so it has come off the tin.
    body_top = ctx.part_world_aabb(body)[1][2]
    ctx.check(
        "lifted lid clears the body rim",
        lifted_aabb[0][2] > body_top - 0.004,
        details=f"lid bottom={lifted_aabb[0][2]:.4f}, body top={body_top:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
