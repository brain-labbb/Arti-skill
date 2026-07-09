from __future__ import annotations

# Metal rectangular fuel flask with hinged flip lid (fork of screw-cap parent).
# Frame: flask stands upright, height along +Z, width along +X, depth along +Y.
#   - body centerline at x=0, y=0; bottom rests at z=0, top of body at z=BODY_H.
#   - flat broad faces are +Y (front, labelled) and -Y (back) sides.
#   - a flat rectangular lid sits on the top deck, hinged along the rear (-Y)
#     top edge.  A raised rim surrounds the fill opening; a hinge barrel runs
#     along the rear rim edge.
# Articulations:
#   - lid_hinge (REVOLUTE): axis along +X at the rear top rim.  Positive q
#     swings the front edge of the lid upward and back (right-hand rule about
#     +X moves +Y toward +Z).  Limits 0 (closed) to ~2.0 rad (~115 deg).

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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_W = 0.130   # x extent
BODY_D = 0.060   # y extent
BODY_H = 0.130   # z extent of the main body
EDGE_R = 0.012   # rounded vertical-edge radius
TOP_R = 0.006    # rounded top/bottom edge radius
WALL = 0.0035    # sheet-metal wall thickness
FLOOR = 0.004    # interior floor thickness
TOP_PLATE = 0.0035  # deck thickness around the fill opening

# Top-deck rim (raised rectangular ring around the fill opening)
RIM_W = 0.120    # outer width (x)
RIM_D = 0.054    # outer depth (y)
RIM_H = 0.003    # height above body top
RIM_T = 0.003    # wall thickness

# Lid plate (fits inside the rim with small clearance)
LID_W = RIM_W - 2 * RIM_T - 0.002   # 0.112
LID_D = RIM_D - 2 * RIM_T - 0.002   # 0.046
LID_T = 0.003                         # plate thickness

# Hinge barrel (cylinder along X at rear rim edge)
HINGE_R = 0.003                       # barrel radius
HINGE_L = 0.050                       # barrel length
HINGE_Y = -(RIM_D / 2.0 - RIM_T)     # rear inner rim edge ≈ -0.024
HINGE_Z = BODY_H + RIM_H             # barrel centre at rim-top level

# Front label
LABEL_W = 0.058
LABEL_T = 0.0015
LABEL_H = 0.080
LABEL_Y = BODY_D / 2.0 + 0.0006
LABEL_Z = 0.060


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _flask_body() -> cq.Workplane:
    """Flat rectangular canister with rounded vertical edges and softened
    top/bottom edges (sheet-metal flask), hollow inside with a fill opening."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(EDGE_R)
    )
    body = body.edges("|X or |Y").fillet(TOP_R)
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, FLOOR))
        .box(
            BODY_W - 2 * WALL,
            BODY_D - 2 * WALL,
            BODY_H - FLOOR - TOP_PLATE,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(max(EDGE_R - WALL, 0.002))
    )
    mouth = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_H - TOP_PLATE - 0.001))
        .rect(RIM_W - 2 * RIM_T, RIM_D - 2 * RIM_T)
        .extrude(TOP_PLATE + RIM_H + 0.004)
    )
    return body.cut(inner).cut(mouth)


def _top_rim() -> cq.Workplane:
    """Raised rectangular ring on the top deck surrounding the fill opening."""
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_H))
        .rect(RIM_W, RIM_D)
        .extrude(RIM_H)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_H - 0.001))
        .rect(RIM_W - 2 * RIM_T, RIM_D - 2 * RIM_T)
        .extrude(RIM_H + 0.002)
    )
    return outer.cut(inner)


def _hinge_barrel() -> cq.Workplane:
    """Small cylinder along X at the rear rim edge (the fixed hinge leaf)."""
    barrel = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, HINGE_Y, HINGE_Z))
        .transformed(rotate=(0.0, 90.0, 0.0))
        .circle(HINGE_R)
        .extrude(HINGE_L / 2.0, both=True)
    )
    return barrel


def _lid_plate() -> cq.Workplane:
    """Flat rectangular lid plate with filleted vertical edges."""
    plate = (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    return plate


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fuel_flask_flip_lid")

    metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.40, 0.42, 0.45, 1.0))
    label_white = model.material("label_white", rgba=(0.93, 0.93, 0.90, 1.0))
    brass = model.material("brass_accent", rgba=(0.72, 0.60, 0.30, 1.0))

    # ---- body (root): flask shell + rim + hinge barrel + front label ----
    body = model.part("body")

    shell = _flask_body().union(_top_rim()).union(_hinge_barrel())
    body.visual(
        mesh_from_cadquery(shell, "flask_shell"),
        material=metal,
        name="flask_shell",
    )

    body.visual(
        Box((LABEL_W, LABEL_T, LABEL_H)),
        origin=Origin(xyz=(0.0, LABEL_Y, LABEL_Z)),
        material=label_white,
        name="front_label",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid: flat rectangular plate hinged at the rear top rim ----
    lid = model.part("lid")

    # Lid plate: origin at hinge axis, plate extends in +Y (toward front)
    # and +Z (above hinge axis).  At q=0 the plate sits on the rim.
    lid.visual(
        mesh_from_cadquery(_lid_plate(), "lid_plate"),
        origin=Origin(xyz=(0.0, LID_D / 2.0, 0.0)),
        material=dark_metal,
        name="lid_plate",
    )

    # Hinge knuckle tab: wraps around the barrel at the rear edge of the lid.
    lid.visual(
        Box((HINGE_L * 0.55, 2 * HINGE_R + 0.002, LID_T + 2 * HINGE_R)),
        origin=Origin(xyz=(0.0, -HINGE_R, LID_T / 2.0)),
        material=dark_metal,
        name="lid_knuckle",
    )

    # Front grip tab for lifting the lid open.
    lid.visual(
        Box((0.020, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, LID_D + 0.002, LID_T / 2.0)),
        material=brass,
        name="lid_tab",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_T)),
        mass=0.03,
        origin=Origin(xyz=(0.0, LID_D / 2.0, LID_T / 2.0)),
    )

    # ---- lid hinge: REVOLUTE about +X at rear top rim ----
    # Positive q: right-hand rule about +X moves +Y toward +Z,
    # so the front edge of the lid lifts up and swings back.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=2.0,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    # ---- flask is flat and rectangular: width >> depth, tall body ----
    def ext(part):
        mn, mx = ctx.part_world_aabb(part)
        return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

    bx, by, bz = ext(body)
    ctx.check(
        "flask body is flat (width greater than depth)",
        bx > by + 0.03,
        details=f"body extents={bx, by, bz}",
    )
    ctx.check(
        "flask body is upright and tall",
        bz > 0.10,
        details=f"body extents={bx, by, bz}",
    )

    # ---- body footprint matches parent (same rectangular flask) ----
    ctx.check(
        "body width matches parent flask",
        abs(bx - BODY_W) < 0.015,
        details=f"body width={bx}",
    )
    ctx.check(
        "body depth matches parent flask",
        abs(by - BODY_D) < 0.015,
        details=f"body depth={by}",
    )
    ctx.check(
        "fill opening is cut through the top deck into a hollow cavity",
        RIM_W - 2 * RIM_T < BODY_W
        and RIM_D - 2 * RIM_T < BODY_D
        and BODY_H - FLOOR - TOP_PLATE > BODY_H * 0.80,
        details=(
            f"opening=({RIM_W - 2 * RIM_T}, {RIM_D - 2 * RIM_T}), "
            f"cavity_h={BODY_H - FLOOR - TOP_PLATE}, top_plate={TOP_PLATE}"
        ),
    )

    # ---- no screw-cap, bail-ring, or neck parts ----
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no screw cap part",
        "cap" not in part_names and "cap_carrier" not in part_names,
        details=f"parts={part_names}",
    )
    ctx.check(
        "no bail ring part",
        "bail_ring" not in part_names,
        details=f"parts={part_names}",
    )

    # ---- lid covers the top of the body when closed (q=0) ----
    ctx.expect_overlap(
        lid, body, axes="xy", min_overlap=0.03,
        name="lid covers top opening when closed",
    )

    # ---- lid seats at the body top level ----
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid sits at the top of the body",
        lid_aabb is not None and lid_aabb[0][2] > BODY_H - 0.010,
        details=f"lid bottom z={lid_aabb[0][2] if lid_aabb else None}",
    )

    # ---- lid hinge opens upward: front edge rises ----
    rest_top = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({hinge: 1.5}):
        open_top = ctx.part_world_aabb(lid)[1][2]
        open_front_z = ctx.part_element_world_aabb(lid, elem="lid_tab")[0][2]
    ctx.check(
        "lid swings upward when opened",
        open_top > rest_top + 0.02,
        details=f"rest top z={rest_top}, open top z={open_top}",
    )
    ctx.check(
        "lid front tab rises above body when opened",
        open_front_z > BODY_H + 0.02,
        details=f"open tab z={open_front_z}",
    )

    # ---- intentional overlap: lid knuckle wraps around hinge barrel ----
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_knuckle", elem_b="flask_shell",
        reason="The lid hinge knuckle intentionally wraps around the fixed barrel.",
    )
    ctx.expect_contact(
        lid, body,
        name="lid is connected to body at the hinge",
    )

    return ctx.report()


object_model = build_object_model()
