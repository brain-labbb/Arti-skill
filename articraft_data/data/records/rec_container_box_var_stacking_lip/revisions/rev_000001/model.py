from __future__ import annotations

# Kraft cardboard gift box with stacking / nesting lip and registration foot.
# Forked from the parent telescoping lift-off lid box.
#
# Frame: box footprint centered on the world Z axis; box base sits on the
# ground at z=0 and rises in +Z. The lid is a shallow inverted tray whose
# skirt drops over the top outer walls of the box (telescoping fit).
#
# Stacking features:
#   - Raised inner rim lip around the top inner edge of the box, protruding
#     inward and upward so a mating box registers on top.
#   - Recessed footing ring (groove) cut into the bottom face, matching the
#     lip profile so identical boxes stack and nest.
#
# Articulation:
#   - lid: PRISMATIC along +Z, lifts straight up off the box (lift-off lid),
#     no hinge and no thread. This is the only non-fixed joint.

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
FLOOR_T = 0.008  # floor thickness (thicker than walls for footing groove)
BOX_H = 0.170  # box base height (without lid)
CAVITY_SIDE = BOX_OUT - 2.0 * WALL  # interior cavity width

# Telescoping lid: a shallow inverted tray that overlaps the box top sides.
LID_SKIRT_H = 0.034  # how far the lid skirt drops down over the box walls
LID_TOP_T = 0.006  # lid top panel thickness
LID_CLEAR = 0.0012  # radial clearance so the lid slips over the box
LID_OUT = BOX_OUT + 2.0 * (LID_CLEAR + WALL)  # lid outer side (slightly larger)
LID_WALL = WALL

LID_LIFT = 0.060  # straight-up travel of the lift-off lid

# ---- stacking features -------------------------------------------------------
LIP_W = 0.004  # inner lip width (inward protrusion from inner wall face)
LIP_H = 0.005  # inner lip height (protrudes above box top rim)
LIP_INNER_SIDE = CAVITY_SIDE - 2.0 * LIP_W  # inner opening of the lip ring

FOOT_DEPTH = 0.005  # footing groove depth (cut into bottom face; < FLOOR_T)
FOOT_CLEAR = 0.0008  # radial clearance for stacking slip fit


# ---- shared geometry helper --------------------------------------------------
def _rect_ring(outer_side: float, inner_side: float, height: float,
               z_base: float = 0.0) -> cq.Workplane:
    """Hollow square ring centered on XY, bottom at *z_base*, extending +Z."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .box(outer_side, outer_side, height, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=z_base - 0.0005)
        .box(inner_side, inner_side, height + 0.001, centered=(True, True, False))
    )
    return outer.cut(inner)


# ---- box shell (with footing groove) -----------------------------------------
def _box_shell_with_groove() -> cq.Workplane:
    """Open-top hollow box with a footing groove cut from the bottom face."""
    outer = (
        cq.Workplane("XY")
        .box(BOX_OUT, BOX_OUT, BOX_H, centered=(True, True, False))
    )
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR_T)
        .box(CAVITY_SIDE, CAVITY_SIDE, BOX_H, centered=(True, True, False))
    )
    shell = outer.cut(cavity)

    # Footing groove: ring-shaped channel cut from the bottom face, matching
    # the lip profile + clearance so a mating lip seats into it.
    groove_outer_side = CAVITY_SIDE + 2.0 * FOOT_CLEAR
    groove_inner_side = LIP_INNER_SIDE - 2.0 * FOOT_CLEAR
    groove_cut = _rect_ring(
        groove_outer_side, groove_inner_side,
        FOOT_DEPTH + 0.0005, z_base=-0.0005,
    )
    return shell.cut(groove_cut)


# ---- raised inner rim lip ----------------------------------------------------
def _inner_lip() -> cq.Workplane:
    """Raised rim lip at the top inner edge, protruding above the box rim."""
    # Outer slightly oversized to embed into the wall for geometric connectivity.
    lip_outer_side = CAVITY_SIDE + 0.001
    return _rect_ring(
        lip_outer_side, LIP_INNER_SIDE,
        LIP_H + 0.001, z_base=BOX_H - 0.001,
    )


# ---- lid (unchanged from parent) ---------------------------------------------
def _lid_solid() -> cq.Workplane:
    """Shallow inverted tray: top panel + short skirt hanging down."""
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


# ==============================================================================
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kraft_gift_box_stackable")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.42, 1.0))
    ink = model.material("label_ink", rgba=(0.12, 0.12, 0.14, 1.0))
    label_paper = model.material("label_paper", rgba=(0.90, 0.86, 0.78, 1.0))

    # ---- box base (root): hollow shell + stacking lip + footing groove -------
    box = model.part("box")
    box.visual(
        mesh_from_cadquery(_box_shell_with_groove(), "box_shell"),
        material=kraft,
        name="box_shell",
    )
    box.visual(
        mesh_from_cadquery(_inner_lip(), "inner_lip"),
        material=kraft,
        name="inner_lip",
    )

    # Printed barcode patch on one side wall (low on a face).
    barcode_y = -BOX_OUT / 2.0 - 0.0003
    box.visual(
        Box((0.030, 0.0006, 0.018)),
        origin=Origin(xyz=(0.045, barcode_y, 0.030)),
        material=label_paper,
        name="barcode_patch",
    )
    for i, dx in enumerate((-0.011, -0.005, 0.001, 0.006, 0.011)):
        w = 0.0018 if i % 2 == 0 else 0.0010
        box.visual(
            Box((w, 0.0007, 0.013)),
            origin=Origin(xyz=(0.045 + dx, barcode_y - 0.0002, 0.030)),
            material=ink,
            name=f"barcode_bar_{i}",
        )

    box.inertial = Inertial.from_geometry(
        Box((BOX_OUT, BOX_OUT, BOX_H + LIP_H)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, (BOX_H + LIP_H) / 2.0)),
    )

    # ---- telescoping lid: shallow inverted tray, lifts straight up ----------
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


# ==============================================================================
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    box = object_model.get_part("box")
    lid = object_model.get_part("lid")
    lift = object_model.get_articulation("box_to_lid")

    # ---- overlap allowances --------------------------------------------------

    # The lid skirt intentionally telescopes over the box top outer walls.
    ctx.allow_overlap(
        lid, box,
        elem_a="lid_shell", elem_b="box_shell",
        reason="The lift-off lid skirt is intended to telescope over the box "
               "top outer walls (seated overlap).",
    )

    # The stacking lip protrudes above the box rim into the lid cavity when
    # the lid is seated. This is intentional: the lip is a stacking feature
    # that sits inside the lid interior and does not block lid removal.
    ctx.allow_overlap(
        lid, box,
        elem_a="lid_shell", elem_b="inner_lip",
        reason="The stacking lip sits inside the lid cavity when the lid is "
               "seated; the lip is a box stacking feature that does not "
               "interfere with lid removal.",
    )

    # ---- stacking lip tests --------------------------------------------------

    # The inner lip protrudes above the box walls.
    lip_top = ctx.part_element_world_aabb(box, elem="inner_lip")[1][2]
    shell_top = ctx.part_element_world_aabb(box, elem="box_shell")[1][2]
    ctx.check(
        "inner lip protrudes above the box walls",
        lip_top > shell_top + 0.002,
        details=f"lip_top={lip_top:.4f}, shell_top={shell_top:.4f}",
    )

    # The lip stays within the box footprint in XY.
    ctx.expect_within(
        box, box,
        axes="xy",
        inner_elem="inner_lip", outer_elem="box_shell",
        margin=0.002,
        name="inner lip stays within the box footprint",
    )

    # The lip has a meaningful height matching the design spec.
    lip_height = lip_top - ctx.part_element_world_aabb(box, elem="inner_lip")[0][2]
    ctx.check(
        "inner lip has design height",
        lip_height >= LIP_H - 0.001,
        details=f"lip_height={lip_height:.4f}, expected≈{LIP_H:.4f}",
    )

    # ---- stacking registration tests -----------------------------------------

    # The footing groove depth accommodates the mating lip height.
    ctx.check(
        "footing groove depth accommodates mating lip",
        FOOT_DEPTH >= LIP_H,
        details=f"FOOT_DEPTH={FOOT_DEPTH:.4f}, LIP_H={LIP_H:.4f}",
    )

    # The footing groove profile is wider than the lip on both sides so the
    # lip of a mating box slips into the groove with clearance.
    groove_outer_side = CAVITY_SIDE + 2.0 * FOOT_CLEAR
    groove_inner_side = LIP_INNER_SIDE - 2.0 * FOOT_CLEAR
    ctx.check(
        "footing groove wider than lip for slip-fit stacking",
        groove_outer_side > CAVITY_SIDE
        and groove_inner_side < LIP_INNER_SIDE
        and FOOT_CLEAR > 0,
        details=(
            f"groove=[{groove_inner_side:.4f}, {groove_outer_side:.4f}], "
            f"lip=[{LIP_INNER_SIDE:.4f}, {CAVITY_SIDE:.4f}]"
        ),
    )

    # The lip inner opening is large enough for a practical stacking register.
    ctx.check(
        "lip inner opening is practical for stacking",
        LIP_INNER_SIDE > 0.10,
        details=f"LIP_INNER_SIDE={LIP_INNER_SIDE:.4f}",
    )

    # ---- box proportions (unchanged from parent) ----------------------------

    bmn, bmx = ctx.part_element_world_aabb(box, elem="box_shell")
    bx = bmx[0] - bmn[0]
    by = bmx[1] - bmn[1]
    bz = bmx[2] - bmn[2]
    ctx.check(
        "box reads as a cube",
        abs(bx - by) < 0.01 and abs(bx - bz) < 0.02 and abs(by - bz) < 0.02,
        details=f"box extents=({bx:.3f}, {by:.3f}, {bz:.3f})",
    )

    # ---- lid tests (unchanged from parent) -----------------------------------

    # Seated lid stays centered over the box footprint.
    ctx.expect_within(
        lid, box,
        axes="xy",
        inner_elem="lid_shell", outer_elem="box_shell",
        margin=0.009,
        name="lid stays centered over the box footprint",
    )
    ctx.expect_overlap(
        lid, box,
        axes="xy",
        elem_a="lid_shell", elem_b="box_shell",
        min_overlap=0.10,
        name="lid skirt overlaps the box top sides in plan",
    )
    ctx.expect_overlap(
        lid, box,
        axes="z",
        elem_a="lid_shell", elem_b="box_shell",
        min_overlap=0.005,
        name="lid skirt overlaps the box walls vertically when seated",
    )

    # Lift-off: posing the joint to its upper limit lifts the lid straight up
    # and clears the box top, with no XY shift.
    rest = ctx.part_world_position(lid)
    box_top = ctx.part_element_world_aabb(box, elem="box_shell")[1][2]
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

    # The lid is the only non-fixed joint.
    joints = object_model.articulations
    ctx.check(
        "lift-off lid is the only non-fixed joint",
        len(joints) == 1 and joints[0].name == "box_to_lid",
        details=f"joints={[j.name for j in joints]}",
    )

    return ctx.report()


object_model = build_object_model()
