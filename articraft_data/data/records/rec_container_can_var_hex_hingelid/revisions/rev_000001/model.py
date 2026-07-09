from __future__ import annotations

# Hexagonal brushed-metal tea tin with a hinged flip lid.
# Fork of the straight-lift-off tea tin: the closure is now a hexagonal cap
# hinged along one rear top edge so it swings up and back (REVOLUTE about a
# horizontal edge axis) instead of lifting straight off.
#
# Frame: hexagonal prism axis along +Z, centered on the world origin in XY.
#   - body: a tall hollow hexagonal-prism tin with a hinge barrel at the
#     rear top edge. Root part, sitting on the ground at z=0.
#   - lid: a hexagonal cap hinged at the rear edge. Swings open upward
#     and back around the hinge barrel.
#
# Scale: ~0.07 m across flats, body ~0.13 m tall.

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
ACROSS_FLATS = 0.070
BODY_CIRCUM = ACROSS_FLATS / (2.0 * math.cos(math.radians(30.0)))
BODY_DIAM = BODY_CIRCUM * 2.0

WALL = 0.0025
BODY_INNER_CIRCUM = BODY_CIRCUM - WALL / math.cos(math.radians(30.0))
BODY_INNER_DIAM = BODY_INNER_CIRCUM * 2.0

BODY_HEIGHT = 0.130
BODY_FLOOR = 0.004

# Lid: slightly larger across-flats so its skirt overhangs the body rim
# on the non-hinge sides.
LID_CLEAR = 0.0012
LID_CIRCUM = BODY_CIRCUM + WALL + LID_CLEAR
LID_DIAM = LID_CIRCUM * 2.0
LID_INNER_CIRCUM = BODY_CIRCUM + LID_CLEAR
LID_INNER_DIAM = LID_INNER_CIRCUM * 2.0

LID_TOP_THICK = 0.003  # thinner top plate than the pull-off lid
LID_SKIRT = 0.012      # skirt depth on non-hinge sides

# Hinge: barrel along the rear top edge, axis along +X.
HINGE_RADIUS = 0.003
HINGE_LENGTH = 0.025
HINGE_OPEN_ANGLE = 2.0  # ~115 degrees

# The polygon(6, D) flat edge at -Y sits at y = -ACROSS_FLATS/2.
BACK_EDGE_Y = -ACROSS_FLATS / 2.0


def _hex_body() -> cq.Workplane:
    """Hollow hexagonal prism body: open top, closed floor at z=0."""
    outer = (
        cq.Workplane("XY")
        .polygon(6, BODY_DIAM)
        .extrude(BODY_HEIGHT)
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=BODY_FLOOR)
        .polygon(6, BODY_INNER_DIAM)
        .extrude(BODY_HEIGHT)
    )
    return outer.cut(cavity)


def _body_with_hinge() -> cq.Workplane:
    """Body shell with a hinge barrel cylinder unioned at the rear top edge.
    The barrel runs along X, centered at (0, BACK_EDGE_Y, BODY_HEIGHT)."""
    body = _hex_body()
    # Cylinder along X: use a YZ workplane whose normal is +X.
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-HINGE_LENGTH / 2.0)
        .center(BACK_EDGE_Y, BODY_HEIGHT)
        .circle(HINGE_RADIUS)
        .extrude(HINGE_LENGTH)
    )
    return body.union(barrel)


def _hex_flip_lid() -> cq.Workplane:
    """Hexagonal flip lid. Local origin at the hinge point (rear edge center
    at the body top). The lid extends forward (+Y) from the hinge.

    The hexagonal profile is shifted so the back flat edge sits at y ≈ 0
    in the lid frame, placing the hinge at the rear of the cap."""
    cy = ACROSS_FLATS / 2.0  # hex center offset from hinge in +Y

    # Top plate: hexagonal cap from z=0 up to z=LID_TOP_THICK.
    top = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, cy)
        .polygon(6, LID_DIAM)
        .extrude(LID_TOP_THICK)
    )

    # Skirt: hollow hex ring hanging below the seam plane (z < 0).
    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT)
        .center(0.0, cy)
        .polygon(6, LID_DIAM)
        .extrude(LID_SKIRT)
    )
    skirt_inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT - 0.001)
        .center(0.0, cy)
        .polygon(6, LID_INNER_DIAM)
        .extrude(LID_SKIRT + 0.002)
    )
    skirt = skirt_outer.cut(skirt_inner)
    lid = top.union(skirt)

    # Cut away the hinge-side (back) portion so the skirt does not wrap
    # behind the hinge axis. Removes everything for y < 0 across the full
    # skirt + top-plate depth.
    back_cut = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT - 0.002)
        .center(0.0, -0.05)
        .rect(0.15, 0.10)
        .extrude(LID_SKIRT + LID_TOP_THICK + 0.004)
    )
    lid = lid.cut(back_cut)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hexagonal_tea_tin")

    body_metal = model.material("body_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    lid_metal = model.material("lid_metal", rgba=(0.78, 0.76, 0.72, 1.0))

    # ---- body (root): hollow hex prism with integral hinge barrel ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_with_hinge(), "tin_body"),
        material=body_metal,
        name="tin_body",
    )
    body.inertial = Inertial.from_geometry(
        Box((ACROSS_FLATS, ACROSS_FLATS, BODY_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    # ---- lid: hexagonal flip cap hinged at the rear edge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_hex_flip_lid(), "tin_lid"),
        material=lid_metal,
        name="tin_lid",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DIAM, ACROSS_FLATS, LID_TOP_THICK + LID_SKIRT)),
        mass=0.04,
        origin=Origin(xyz=(0.0, ACROSS_FLATS / 2.0,
                           (LID_TOP_THICK - LID_SKIRT) / 2.0)),
    )

    # Revolute hinge at the rear top edge. Axis along +X so positive q
    # rotates the lid front upward (right-hand rule: +Y toward +Z).
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BACK_EDGE_Y, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=HINGE_OPEN_ANGLE,
            effort=5.0,
            velocity=2.0,
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
    hinge = object_model.get_articulation("body_to_lid")

    # --- Body shape: hexagonal prism, taller than wide ---
    body_ext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body is taller than wide (tall hex prism)",
        body_ext[2] > max(body_ext[0], body_ext[1]) + 0.03,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "body across-flats footprint near 0.07 m",
        0.06 < max(body_ext[0], body_ext[1]) < 0.085,
        details=f"body extents={body_ext}",
    )
    ctx.check(
        "cross-section is hexagonal, not square/round",
        abs(body_ext[0] - body_ext[1]) > 0.003,
        details=f"x={body_ext[0]:.4f}, y={body_ext[1]:.4f}",
    )

    # --- Joint is revolute (flip lid), not prismatic ---
    ctx.check(
        "lid joint is revolute (hinged flip)",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type={hinge.articulation_type}",
    )

    # --- Closed pose: lid skirt seated over body rim ---
    # The skirt intentionally overlaps the outside of the body wall, and the
    # lid top plate wraps around the hinge barrel at the rear pivot.
    ctx.allow_overlap(
        lid, body,
        elem_a="tin_lid",
        elem_b="tin_body",
        reason=(
            "The lid skirt seats over the body rim when closed, and the lid "
            "top plate wraps around the hinge barrel at the rear pivot."
        ),
    )
    ctx.expect_overlap(
        lid, body,
        axes="z",
        min_overlap=0.004,
        name="lid skirt overlaps body rim when closed",
    )
    ctx.expect_overlap(
        lid, body,
        axes="xy",
        min_overlap=0.04,
        name="lid covers body top when closed",
    )

    # --- Open pose: lid swings up and back around the hinge ---
    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: HINGE_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens upward (AABB top rises)",
        open_aabb[1][2] > rest_aabb[1][2] + 0.02,
        details=f"rest_top={rest_aabb[1][2]:.4f}, open_top={open_aabb[1][2]:.4f}",
    )
    ctx.check(
        "lid swings back past vertical (AABB y_min retreats behind body)",
        open_aabb[0][1] < rest_aabb[0][1] - 0.01,
        details=f"rest_ymin={rest_aabb[0][1]:.4f}, open_ymin={open_aabb[0][1]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
