from __future__ import annotations

# Round brushed-metal canister tin with a flat lift-off lid.
# Frame: tin axis along +Z, body base sitting on z=0, centerline at x=y=0.
# Body is a hollow cylinder (open top) taller than it is wide.
# The lid is a flat cap with a short downward skirt that overhangs the body
# rim (the visible seam ring). The lid lifts straight up off the tin.
# Articulations:
#   - lid: PRISMATIC, travels straight up (+Z) ~0.04 m off the body.

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

# ---- canister dimensions (meters) ----
BODY_R = 0.050  # outer radius of the tin body (~0.10 m diameter)
BODY_H = 0.118  # body height (rim sits at z = BODY_H)
WALL = 0.0035  # sheet-metal wall thickness
FLOOR = 0.004  # interior floor thickness

LID_R = 0.0515  # lid slightly proud of the body so the skirt overhangs the rim
LID_TOP_H = 0.010  # thickness of the flat lid top plate
SKIRT_H = 0.018  # how far the lid skirt drops down over the body rim
SKIRT_WALL = 0.003  # skirt wall thickness

# The lid skirt drops over the body rim; this is the seated overlap depth.
SEAT_OVERLAP = SKIRT_H - LID_TOP_H * 0.0  # full skirt overhangs the rim
RIM_TOP_Z = BODY_H  # body rim top in body frame


def _body_solid() -> cq.Workplane:
    # Hollow cylinder, open at the top: outer cylinder minus an inner bore that
    # stops short of the floor and runs out past the top so the rim is a real
    # open ring.
    outer = cq.Workplane("XY").circle(BODY_R).extrude(BODY_H)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR)
        .circle(BODY_R - WALL)
        .extrude(BODY_H)  # overshoots the top so the rim is open
    )
    body = outer.cut(inner)
    return body


def _lid_solid() -> cq.Workplane:
    # Flat top plate plus a thin downward skirt ring that caps over the body rim.
    # Built in the lid's own local frame with the underside of the skirt at z=0
    # so the lid's mount origin can sit on the body rim.
    # Skirt: a hollow ring (outer LID_R, inner = body outer radius + clearance).
    skirt_inner = BODY_R + 0.0008  # tiny clearance so the skirt sleeves the rim
    skirt = (
        cq.Workplane("XY")
        .circle(LID_R)
        .circle(skirt_inner)
        .extrude(SKIRT_H)
    )
    # Flat top plate closing the top of the skirt.
    top = (
        cq.Workplane("XY")
        .workplane(offset=SKIRT_H)
        .circle(LID_R)
        .extrude(LID_TOP_H)
    )
    lid = skirt.union(top)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="canister_tin")

    brushed = model.material("brushed_metal", rgba=(0.80, 0.81, 0.83, 1.0))

    # ---- body (root): hollow brushed-metal tin ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "body_shell"),
        material=brushed,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_H),
        mass=0.090,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # ---- lid: flat cap with overhanging skirt, lifts straight up ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=brushed,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(LID_R, SKIRT_H + LID_TOP_H),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, (SKIRT_H + LID_TOP_H) / 2.0)),
    )

    # Seated so the skirt sleeves down over the body rim. The lid local z=0 is the
    # bottom of the skirt; placing the mount origin below the rim top by
    # SEAT_OVERLAP makes the skirt overhang the rim (the visible seam ring).
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z - SEAT_OVERLAP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.1, lower=0.0, upper=0.040),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("body_to_lid")

    # Body reads as a cylinder taller than it is wide.
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "body taller than wide",
        bext[2] > max(bext[0], bext[1]) + 0.01,
        details=f"body extents={bext}",
    )
    ctx.check(
        "body roughly circular in plan",
        abs(bext[0] - bext[1]) < 0.005,
        details=f"body extents={bext}",
    )

    # Lid skirt intentionally overhangs / sleeves the body rim when seated (seam ring).
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="body_shell",
        reason="Lid skirt sleeves down over the body rim when the lid is seated (visible seam ring).",
    )
    ctx.expect_overlap(
        lid, body, axes="z", min_overlap=0.004, name="lid skirt overlaps body rim when down"
    )

    # Lid sits at the top of the tin.
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid mounted at the tin top",
        lid_pos is not None and lid_pos[2] > BODY_H - SKIRT_H - 1e-6,
        details=f"lid origin={lid_pos}",
    )

    # Lifting the lid raises it straight up and clears the body rim (no overlap).
    z_rest = ctx.part_world_aabb(lid)[0][2]  # min-z of lid when seated
    with ctx.pose({lid_joint: 0.040}):
        z_lift = ctx.part_world_aabb(lid)[0][2]
        lifted_ext = _ext(ctx.part_world_aabb(lid))
    ctx.check(
        "lid lifts straight up off the tin",
        z_lift > z_rest + 0.035,
        details=f"z_rest={z_rest}, z_lift={z_lift}",
    )
    # When fully lifted, the skirt bottom clears the body rim top (lid is off).
    ctx.check(
        "lifted lid clears the body rim",
        z_lift > RIM_TOP_Z - 1e-4,
        details=f"z_lift={z_lift}, rim_top={RIM_TOP_Z}",
    )
    # Lid stays the same flat cap shape (wider than it is tall).
    ctx.check(
        "lid is a flat cap",
        max(lifted_ext[0], lifted_ext[1]) > lifted_ext[2] + 0.01,
        details=f"lid extents={lifted_ext}",
    )

    return ctx.report()


object_model = build_object_model()
