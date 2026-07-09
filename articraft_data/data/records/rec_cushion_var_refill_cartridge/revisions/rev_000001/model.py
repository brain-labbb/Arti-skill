from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


def _hollow_bowl() -> cq.Workplane:
    """Thick translucent smoked-acrylic bowl with an inner recess."""
    outer = cq.Workplane("XY").cylinder(0.022, 0.068).translate((0.0, 0.0, 0.011))
    cavity = cq.Workplane("XY").cylinder(0.018, 0.056).translate((0.0, 0.0, 0.017))
    return outer.cut(cavity)


def _domed_lid() -> cq.Workplane:
    """White domed lid with a rounded top edge."""
    return cq.Workplane("XY").cylinder(0.012, 0.067).edges(">Z").fillet(0.006)


def _refill_pan() -> cq.Workplane:
    """Shallow hollow refill cartridge pan that lifts out of the bowl."""
    outer = cq.Workplane("XY").cylinder(0.010, 0.054).translate((0.0, 0.0, 0.005))
    cavity = cq.Workplane("XY").cylinder(0.008, 0.049).translate((0.0, 0.0, 0.006))
    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_cushion_compact")

    clear = _mat(model, "smoked_translucent_acrylic", (0.46, 0.42, 0.37, 0.38))
    white = _mat(model, "smooth_white", (0.93, 0.92, 0.88, 1.0))
    black = _mat(model, "black_gasket", (0.02, 0.02, 0.02, 1.0))
    mirror = _mat(model, "round_mirror", (0.72, 0.78, 0.80, 0.55))
    powder = _mat(model, "beige_powder", (0.86, 0.69, 0.54, 1.0))
    ink = _mat(model, "label_ink", (0.18, 0.20, 0.22, 1.0))
    pink = _mat(model, "pink_emblem", (0.72, 0.20, 0.38, 1.0))
    metal = _mat(model, "hinge_metal", (0.56, 0.55, 0.52, 1.0))
    refill_plastic = _mat(model, "refill_plastic", (0.90, 0.88, 0.82, 1.0))
    sponge_foam = _mat(model, "sponge_foam", (0.94, 0.87, 0.80, 1.0))

    # ----- Translucent bowl base (root) -----
    base = model.part("base")
    base.visual(mesh_from_cadquery(_hollow_bowl(), "translucent_bowl"), material=clear, name="clear_bowl")
    base.visual(Cylinder(radius=0.058, length=0.003), origin=Origin(xyz=(0.0, 0.0, 0.022)), material=black, name="rim_gasket")
    base.visual(Box((0.032, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.066, 0.016)), material=black, name="front_latch_socket")
    base.visual(Box((0.072, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.063, 0.024)), material=metal, name="rear_hinge_leaf")
    for side, x in enumerate((-0.026, 0.026)):
        base.visual(Box((0.022, 0.014, 0.016)), origin=Origin(xyz=(x, 0.066, 0.024)), material=metal, name=f"hinge_saddle_{side}")
        base.visual(Cylinder(radius=0.005, length=0.026), origin=Origin(xyz=(x, 0.066, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)), material=metal, name=f"hinge_barrel_{side}")

    # ----- Removable refill cartridge (prismatic child of base) -----
    refill = model.part("refill")
    refill.visual(mesh_from_cadquery(_refill_pan(), "refill_pan_shell"), material=refill_plastic, name="refill_pan")
    # Powder fill sits on the pan cavity floor with a slight embed for connectivity.
    refill.visual(Cylinder(radius=0.047, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=powder, name="powder_fill")
    # Soft sponge pad on top of the powder surface.
    refill.visual(Cylinder(radius=0.044, length=0.005), origin=Origin(xyz=(0.0, 0.0, 0.0095)), material=sponge_foam, name="sponge")
    # Locating tabs around the pan rim for rotational alignment in the bowl.
    for i in range(4):
        angle = i * math.pi / 2.0
        tx = 0.053 * math.cos(angle)
        ty = 0.053 * math.sin(angle)
        refill.visual(
            Box((0.006, 0.004, 0.008)),
            origin=Origin(xyz=(tx, ty, 0.005)),
            material=refill_plastic,
            name=f"locating_tab_{i}",
        )

    model.articulation(
        "base_to_refill",
        ArticulationType.PRISMATIC,
        parent=base,
        child=refill,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.10, lower=0.0, upper=0.040),
    )

    # ----- White domed lid (child) authored in rear-hinge pivot frame -----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_domed_lid(), "white_domed_lid"), origin=Origin(xyz=(0.0, -0.066, 0.006)), material=white, name="white_domed_lid")
    lid.visual(Cylinder(radius=0.048, length=0.003), origin=Origin(xyz=(0.0, -0.066, 0.013)), material=white, name="label_disc")
    lid.visual(Cylinder(radius=0.043, length=0.0015), origin=Origin(xyz=(0.0, -0.066, 0.0145)), material=ink, name="printed_label_ring")
    # Small geometric flower emblem in the label center.
    lid.visual(Cylinder(radius=0.006, length=0.002), origin=Origin(xyz=(0.0, -0.066, 0.015)), material=pink, name="flower_core")
    for i, a in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        lid.visual(Cylinder(radius=0.004, length=0.002), origin=Origin(xyz=(0.013 * math.cos(a), -0.066 + 0.013 * math.sin(a), 0.015)), material=pink, name=f"flower_petal_{i}")
    # Large mirror under the lid.
    lid.visual(Cylinder(radius=0.055, length=0.0025), origin=Origin(xyz=(0.0, -0.066, -0.0010)), material=mirror, name="inner_mirror")
    # Front latch tab.
    lid.visual(Box((0.030, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.134, 0.002)), material=black, name="front_latch_tab")
    for side, x in enumerate((-0.026, 0.026)):
        lid.visual(Cylinder(radius=0.0035, length=0.020), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=metal, name=f"lid_hinge_pin_{side}")

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.066, 0.030)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.3, lower=0.0, upper=1.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    refill = object_model.get_part("refill")
    hinge = object_model.get_articulation("base_to_lid")
    slide = object_model.get_articulation("base_to_refill")

    # Hinge pin / barrel / saddle overlap allowances
    for side in (0, 1):
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_barrel_{side}", reason="The lid hinge pin is captured by the rear hinge barrel.")
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_saddle_{side}", reason="The lid hinge pin passes through the hinge saddle mounting block.")
        ctx.allow_overlap(base, lid, elem_a=f"hinge_barrel_{side}", elem_b="white_domed_lid", reason="The rear hinge barrel is locally nested under the rear lid edge.")

    # Refill cartridge sits inside the bowl cavity — intentional overlap of
    # the pan walls with the bowl interior.
    ctx.allow_overlap(refill, base, elem_a="refill_pan", elem_b="clear_bowl", reason="The refill cartridge pan is seated inside the translucent bowl cavity.")
    for i in range(4):
        ctx.allow_overlap(refill, base, elem_a=f"locating_tab_{i}", elem_b="clear_bowl", reason=f"Locating tab {i} protrudes into the bowl wall for rotational alignment.")

    # --- Closed-pose checks ---
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.080, name="closed lid covers the bowl footprint")
    ctx.expect_overlap(lid, lid, axes="xy", elem_a="inner_mirror", elem_b="white_domed_lid", min_overlap=0.090, name="mirror fills the inside of the lid")
    ctx.expect_overlap(lid, base, axes="x", elem_a="front_latch_tab", elem_b="front_latch_socket", min_overlap=0.020, name="front latch lines up with the socket")

    # Refill seated inside the bowl
    ctx.expect_within(refill, base, axes="xy", inner_elem="refill_pan", outer_elem="clear_bowl", margin=0.005, name="refill pan stays within the bowl footprint")
    ctx.expect_within(refill, base, axes="z", inner_elem="refill_pan", outer_elem="clear_bowl", margin=0.002, name="refill pan stays within the bowl cavity height")

    # --- Lid opens upward ---
    closed_lid = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    with ctx.pose({hinge: 1.20}):
        opened_lid = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    ctx.check("lid opens upward", closed_lid is not None and opened_lid is not None and opened_lid[1][2] > closed_lid[1][2] + 0.040, details=f"closed={closed_lid}, opened={opened_lid}")

    # --- Refill lifts upward on prismatic slide ---
    seated_pos = ctx.part_world_position(refill)
    with ctx.pose({slide: 0.035}):
        lifted_pos = ctx.part_world_position(refill)
    ctx.check(
        "refill lifts straight up",
        seated_pos is not None and lifted_pos is not None and lifted_pos[2] > seated_pos[2] + 0.025,
        details=f"seated={seated_pos}, lifted={lifted_pos}",
    )

    # Refill retains XY alignment when lifted
    with ctx.pose({slide: 0.035}):
        ctx.expect_within(refill, base, axes="xy", inner_elem="refill_pan", outer_elem="clear_bowl", margin=0.010, name="lifted refill stays roughly above the bowl")

    # --- Visual count sanity ---
    ctx.check(
        "refill has pan, powder, sponge and tabs",
        len(refill.visuals) >= 6,
        details=f"refill visuals: {[v.name for v in refill.visuals]}",
    )
    ctx.check(
        "round compact has hollow bowl, label and mirror",
        len(base.visuals) >= 5 and len(lid.visuals) >= 9,
        details="base should be a hollow translucent bowl with hinge; lid should include label, flower, mirror and latch",
    )
    return ctx.report()


object_model = build_object_model()
