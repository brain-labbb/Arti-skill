from __future__ import annotations

# Square brushed-metal tin box with a press-fit, lift-off lid.
# Frame: box centered on the world Z axis, footprint in XY, ground at z=0.
#   - The rounded-square TIN BODY is the root: a hollow shell with a raised rim.
#   - The flat LID seats on top with an overhanging skirt that wraps the rim.
# Articulations:
#   - tin_to_lid: PRISMATIC, lid lifts straight up (+Z) off the tin (~0.04 m).
#     When seated (pose 0) the lid skirt overlaps the body rim (press fit).

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

# ---- key dimensions (meters) ----
HALF = 0.060          # half-width of the square footprint (0.12 m wide)
FILLET = 0.012        # corner radius of the rounded square
WALL = 0.0035         # body wall thickness

BODY_BOTTOM = 0.0     # body sits on the ground
RIM_TOP = 0.108       # top of the body rim (where the lid seats)
BODY_OUTER_HALF = HALF
RIM_INSET = 0.004     # the rim step is set in slightly from the outer wall

LID_SEATED_BASE = 0.104  # underside of the lid top plate when seated
LID_TOP = 0.120          # top of the lid (overall height ~0.12)
SKIRT_DROP = 0.012       # how far the lid skirt hangs down over the rim
LID_OUTER_HALF = HALF + 0.0015  # lid overhangs the body wall slightly
LID_LIFT = 0.040         # prismatic travel for lifting the lid off


def _rounded_square_prism(half: float, fillet: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(2.0 * half, 2.0 * half)
        .extrude(z1 - z0)
        .edges("|Z")
        .fillet(fillet)
    )


def _body_solid() -> cq.Workplane:
    # Hollow rounded-square shell: outer prism minus an inner cavity, leaving a
    # solid floor and four walls. A slightly inset raised rim caps the walls so
    # the lid skirt can wrap around it.
    outer = _rounded_square_prism(BODY_OUTER_HALF, FILLET, BODY_BOTTOM, RIM_TOP)

    # Inner cavity: smaller rounded square, open at the top, leaving a floor.
    inner_half = BODY_OUTER_HALF - WALL
    inner = _rounded_square_prism(
        inner_half, max(FILLET - WALL, 0.002), WALL, RIM_TOP + 0.001
    )
    shell = outer.cut(inner)

    # Rim rabbet: shave a shallow band off the very top outer edge so the rim is
    # inset and the lid skirt can seat against an inset rim shoulder (the visible
    # seam line in the reference image).
    rabbet_outer = _rounded_square_prism(
        BODY_OUTER_HALF + 0.001, FILLET, RIM_TOP - 0.014, RIM_TOP + 0.001
    )
    rabbet_inner = _rounded_square_prism(
        BODY_OUTER_HALF - RIM_INSET, max(FILLET - RIM_INSET, 0.002),
        RIM_TOP - 0.014, RIM_TOP + 0.001,
    )
    rabbet = rabbet_outer.cut(rabbet_inner)
    shell = shell.cut(rabbet)
    return shell


def _lid_solid() -> cq.Workplane:
    # Flat lid: a top plate plus a thin overhanging skirt that drops down over
    # the body rim. Modeled in the lid's local frame; the joint places it.
    plate = _rounded_square_prism(LID_OUTER_HALF, FILLET, LID_SEATED_BASE, LID_TOP)

    # Skirt: a thin downward wall ring that wraps the inset rim of the body.
    skirt_z1 = LID_SEATED_BASE + 0.001
    skirt_z0 = LID_SEATED_BASE - SKIRT_DROP
    skirt_outer = _rounded_square_prism(LID_OUTER_HALF, FILLET, skirt_z0, skirt_z1)
    skirt_inner = _rounded_square_prism(
        BODY_OUTER_HALF - RIM_INSET - 0.0008,
        max(FILLET - RIM_INSET, 0.002),
        skirt_z0 - 0.001,
        skirt_z1 + 0.001,
    )
    skirt = skirt_outer.cut(skirt_inner)

    return plate.union(skirt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_tin_box")

    brushed = model.material("brushed_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    brushed_lid = model.material("brushed_metal_lid", rgba=(0.70, 0.71, 0.73, 1.0))

    # ---- tin body (root) ----
    body = model.part("tin_body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "tin_body_shell"),
        material=brushed,
        name="tin_body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((2 * HALF, 2 * HALF, RIM_TOP)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- press-fit lid (lifts straight up) ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=brushed_lid,
        name="lid_shell",
    )
    lid_h = LID_TOP - (LID_SEATED_BASE - SKIRT_DROP)
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_OUTER_HALF, 2 * LID_OUTER_HALF, lid_h)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, (LID_TOP + LID_SEATED_BASE - SKIRT_DROP) / 2.0)),
    )

    model.articulation(
        "tin_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.1, lower=0.0, upper=LID_LIFT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tin_body")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("tin_to_lid")

    # The lid skirt presses over the body rim when seated: a local, intentional
    # overlap between the lid skirt and the body rim band.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="tin_body_shell",
        reason="Press-fit lid skirt intentionally wraps over the inset body rim when seated.",
    )

    # Tin is a rounded square: footprint nearly square in X and Y.
    bmn, bmx = ctx.part_world_aabb(body)
    bx = bmx[0] - bmn[0]
    by = bmx[1] - bmn[1]
    bz = bmx[2] - bmn[2]
    ctx.check(
        "tin body has a square footprint",
        abs(bx - by) < 0.004 and bx > 0.10,
        details=f"footprint x={bx:.4f} y={by:.4f}",
    )
    ctx.check(
        "tin body is roughly a cube",
        bz > 0.09 and bz < 0.13,
        details=f"body height z={bz:.4f}",
    )

    # Seated lid sits on top of the body and its skirt overlaps the rim.
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="tin_body_shell",
        min_overlap=0.08,
        name="seated lid covers the tin footprint",
    )

    lmn0, lmx0 = ctx.part_world_aabb(lid)
    ctx.check(
        "seated lid skirt drops over the body rim",
        lmn0[2] < RIM_TOP - 0.004,
        details=f"lid bottom z={lmn0[2]:.4f}, rim top={RIM_TOP:.4f}",
    )
    ctx.check(
        "seated lid sits at the top of the tin",
        lmx0[2] > RIM_TOP,
        details=f"lid top z={lmx0[2]:.4f}",
    )

    # Lifting the lid moves it straight up and clears the body rim.
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({lift: LID_LIFT}):
        lifted_z = ctx.part_world_position(lid)[2]
        lmn1, _ = ctx.part_world_aabb(lid)
    ctx.check(
        "lid lifts straight up off the tin",
        lifted_z > rest_z + 0.03,
        details=f"rest_z={rest_z:.4f}, lifted_z={lifted_z:.4f}",
    )
    ctx.check(
        "lifted lid clears the body rim",
        lmn1[2] > RIM_TOP - 0.001,
        details=f"lifted lid bottom z={lmn1[2]:.4f}, rim top={RIM_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
