from __future__ import annotations

# Kraft cardboard gift box with a separate telescoping lift-off lid.
# Frame: box footprint centered on the world Z axis; box base sits on the
# ground at z=0 and rises in +Z. The lid is a shallow inverted tray whose
# skirt drops over the top outer walls of the box (telescoping fit).
# Articulation:
#   - lid: PRISMATIC along +Z, lifts straight up off the box (lift-off lid),
#     no hinge and no thread.

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

# ---- overall dimensions (a roughly 0.18 m kraft cube) -----------------------
BOX_OUT = 0.180  # outer side of the box (square footprint)
WALL = 0.006  # cardboard wall thickness
BOX_H = 0.170  # box base height (without lid)

# Telescoping lid: a shallow inverted tray that overlaps the box top sides.
LID_SKIRT_H = 0.034  # how far the lid skirt drops down over the box walls
LID_TOP_T = 0.006  # lid top panel thickness
LID_CLEAR = 0.0012  # radial clearance so the lid slips over the box
LID_OUT = BOX_OUT + 2.0 * (LID_CLEAR + WALL)  # lid outer side (slightly larger)
LID_WALL = WALL

LID_LIFT = 0.060  # straight-up travel of the lift-off lid


def _box_base_solid() -> cq.Workplane:
    # Open-top hollow shell: four walls + a bottom, hollow inside.
    outer = (
        cq.Workplane("XY")
        .box(BOX_OUT, BOX_OUT, BOX_H, centered=(True, True, False))
    )
    # Carve out the interior, leaving the bottom floor and open at the top.
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            BOX_OUT - 2.0 * WALL,
            BOX_OUT - 2.0 * WALL,
            BOX_H,  # tall enough to break through the open top
            centered=(True, True, False),
        )
    )
    return outer.cut(cavity)


def _lid_solid() -> cq.Workplane:
    # Shallow inverted tray: a top panel with a short skirt hanging down.
    # Modeled in its own local frame with the top panel at z=0 and the skirt
    # going to -Z; the articulation origin places it above the box.
    top = (
        cq.Workplane("XY")
        .box(LID_OUT, LID_OUT, LID_TOP_T, centered=(True, True, False))
    )
    skirt_outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_H)
        .box(LID_OUT, LID_OUT, LID_SKIRT_H, centered=(True, True, False))
    )
    skirt_cavity = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_H - 0.001)
        .box(
            LID_OUT - 2.0 * LID_WALL,
            LID_OUT - 2.0 * LID_WALL,
            LID_SKIRT_H + 0.002,
            centered=(True, True, False),
        )
    )
    skirt = skirt_outer.cut(skirt_cavity)
    return top.union(skirt)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kraft_gift_box")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.42, 1.0))
    ink = model.material("label_ink", rgba=(0.12, 0.12, 0.14, 1.0))
    label_paper = model.material("label_paper", rgba=(0.90, 0.86, 0.78, 1.0))

    # ---- box base (root): open-top hollow kraft shell --------------------
    box = model.part("box")
    box.visual(
        mesh_from_cadquery(_box_base_solid(), "box_shell"),
        material=kraft,
        name="box_shell",
    )

    # Printed barcode patch on one side wall (low on a face, as in the image).
    barcode_y = -BOX_OUT / 2.0 - 0.0003
    box.visual(
        Box((0.030, 0.0006, 0.018)),
        origin=Origin(xyz=(0.045, barcode_y, 0.030)),
        material=label_paper,
        name="barcode_patch",
    )
    # A few barcode bars on the patch.
    for i, dx in enumerate((-0.011, -0.005, 0.001, 0.006, 0.011)):
        w = 0.0018 if i % 2 == 0 else 0.0010
        box.visual(
            Box((w, 0.0007, 0.013)),
            origin=Origin(xyz=(0.045 + dx, barcode_y - 0.0002, 0.030)),
            material=ink,
            name=f"barcode_bar_{i}",
        )

    box.inertial = Inertial.from_geometry(
        Box((BOX_OUT, BOX_OUT, BOX_H)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, BOX_H / 2.0)),
    )

    # ---- telescoping lid: shallow inverted tray, lifts straight up -------
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=kraft,
        name="lid_shell",
    )

    # Small diamond paper label centered on the lid top.
    diamond = (
        cq.Workplane("XY")
        .workplane(offset=LID_TOP_T)
        .polyline([(0.0, 0.020), (0.014, 0.0), (0.0, -0.020), (-0.014, 0.0)])
        .close()
        .extrude(0.0007)
    )
    lid.visual(
        mesh_from_cadquery(diamond, "diamond_label"),
        material=label_paper,
        name="diamond_label",
    )
    # A couple of ink lines across the diamond label.
    for j, dz in enumerate((0.006, -0.006)):
        lid.visual(
            Box((0.016, 0.012, 0.0006)),
            origin=Origin(xyz=(0.0, dz, LID_TOP_T + 0.0008)),
            material=ink,
            name=f"diamond_line_{j}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((LID_OUT, LID_OUT, LID_SKIRT_H + LID_TOP_T)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -LID_SKIRT_H / 2.0)),
    )

    # Seated lid: its top panel rests on the box top rim. Lid local origin is
    # at the top panel underside (z=0), so place the joint origin at the box
    # top so the skirt drops down over the walls.
    model.articulation(
        "box_to_lid",
        ArticulationType.PRISMATIC,
        parent=box,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, BOX_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.2, lower=0.0, upper=LID_LIFT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    box = object_model.get_part("box")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("box_to_lid")

    # The lid skirt intentionally telescopes over the box top outer walls.
    ctx.allow_overlap(
        lid,
        box,
        elem_a="lid_shell",
        elem_b="box_shell",
        reason="The lift-off lid skirt is intended to telescope over the box "
        "top outer walls (seated overlap).",
    )

    # Box is a cube: outer extents roughly equal on all three axes.
    bmn, bmx = ctx.part_element_world_aabb(box, elem="box_shell")
    bx = bmx[0] - bmn[0]
    by = bmx[1] - bmn[1]
    bz = bmx[2] - bmn[2]
    ctx.check(
        "box reads as a cube",
        abs(bx - by) < 0.01 and abs(bx - bz) < 0.02 and abs(by - bz) < 0.02,
        details=f"box extents=({bx:.3f}, {by:.3f}, {bz:.3f})",
    )

    # Seated lid: its skirt stays centered over the box footprint and overlaps
    # the box top walls in plan view (retained telescoping fit).
    # The lid is intentionally a touch larger than the box so its skirt drops
    # over the outside; allow that small overhang in the centering margin.
    ctx.expect_within(
        lid,
        box,
        axes="xy",
        inner_elem="lid_shell",
        outer_elem="box_shell",
        margin=0.009,
        name="lid stays centered over the box footprint",
    )
    ctx.expect_overlap(
        lid,
        box,
        axes="xy",
        elem_a="lid_shell",
        elem_b="box_shell",
        min_overlap=0.10,
        name="lid skirt overlaps the box top sides in plan",
    )
    # The lid skirt actually drops down alongside the box walls when seated.
    ctx.expect_overlap(
        lid,
        box,
        axes="z",
        elem_a="lid_shell",
        elem_b="box_shell",
        min_overlap=0.005,
        name="lid skirt overlaps the box walls vertically when seated",
    )

    # Lift-off: posing the joint to its upper limit lifts the lid straight up
    # and clears the box top, with no XY shift.
    rest = ctx.part_world_position(lid)
    rest_box = box_top = ctx.part_element_world_aabb(box, elem="box_shell")[1][2]
    with ctx.pose({lift: LID_LIFT}):
        lifted = ctx.part_world_position(lid)
        lid_bottom = ctx.part_element_world_aabb(lid, elem="lid_shell")[0][2]
    ctx.check(
        "lid lifts straight up off the box",
        lifted[2] > rest[2] + 0.05
        and abs(lifted[0] - rest[0]) < 1e-4
        and abs(lifted[1] - rest[1]) < 1e-4,
        details=f"rest={rest}, lifted={lifted}",
    )
    ctx.check(
        "lifted lid clears the box top",
        lid_bottom >= box_top - 0.001,
        details=f"lid_bottom={lid_bottom:.3f}, box_top={box_top:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
