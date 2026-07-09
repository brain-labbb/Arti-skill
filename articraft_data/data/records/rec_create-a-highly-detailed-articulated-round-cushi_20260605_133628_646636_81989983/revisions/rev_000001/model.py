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

    # ----- Translucent bowl base (root) -----
    base = model.part("base")
    base.visual(mesh_from_cadquery(_hollow_bowl(), "translucent_bowl"), material=clear, name="clear_bowl")
    base.visual(Cylinder(radius=0.058, length=0.003), origin=Origin(xyz=(0.0, 0.0, 0.022)), material=black, name="rim_gasket")
    base.visual(Cylinder(radius=0.050, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=powder, name="powder_pan")
    base.visual(Cylinder(radius=0.040, length=0.003), origin=Origin(xyz=(0.0, 0.0, 0.023)), material=powder, name="powder_dome")
    base.visual(Box((0.032, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.066, 0.016)), material=black, name="front_latch_socket")
    base.visual(Box((0.072, 0.012, 0.010)), origin=Origin(xyz=(0.0, 0.063, 0.024)), material=metal, name="rear_hinge_leaf")
    for side, x in enumerate((-0.026, 0.026)):
        base.visual(Box((0.022, 0.014, 0.016)), origin=Origin(xyz=(x, 0.066, 0.024)), material=metal, name=f"hinge_saddle_{side}")
        base.visual(Cylinder(radius=0.005, length=0.026), origin=Origin(xyz=(x, 0.066, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)), material=metal, name=f"hinge_barrel_{side}")

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
    hinge = object_model.get_articulation("base_to_lid")

    for side in (0, 1):
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_barrel_{side}", reason="The lid hinge pin is captured by the rear hinge barrel.")
        ctx.allow_overlap(base, lid, elem_a=f"hinge_barrel_{side}", elem_b="white_domed_lid", reason="The rear hinge barrel is locally nested under the rear lid edge.")

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.080, name="closed lid covers the bowl footprint")
    ctx.expect_overlap(base, base, axes="xy", elem_a="powder_pan", elem_b="clear_bowl", min_overlap=0.080, name="powder pan seated inside the translucent bowl")
    ctx.expect_overlap(lid, lid, axes="xy", elem_a="inner_mirror", elem_b="white_domed_lid", min_overlap=0.090, name="mirror fills the inside of the lid")
    ctx.expect_overlap(lid, base, axes="x", elem_a="front_latch_tab", elem_b="front_latch_socket", min_overlap=0.020, name="front latch lines up with the socket")

    closed = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    with ctx.pose({hinge: 1.20}):
        opened = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    ctx.check("lid opens upward", closed is not None and opened is not None and opened[1][2] > closed[1][2] + 0.040, details=f"closed={closed}, opened={opened}")
    ctx.check("round compact has hollow bowl, label and mirror", len(base.visuals) >= 6 and len(lid.visuals) >= 9, details="base should be a hollow translucent bowl with powder pan and hinge; lid should include label, flower, mirror and latch")
    return ctx.report()


object_model = build_object_model()
