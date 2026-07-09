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


def _puff_tray_disc() -> cq.Workplane:
    """Thin shallow round tray with a small raised lip rim."""
    disc = cq.Workplane("XY").circle(0.048).extrude(0.0018)
    rim = cq.Workplane("XY").workplane(offset=0.0010).circle(0.050).circle(0.047).extrude(0.0015)
    return disc.union(rim).edges(">Z").fillet(0.0004)


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
    sponge = _mat(model, "soft_sponge", (0.95, 0.90, 0.82, 1.0))
    tray_white = _mat(model, "tray_white", (0.91, 0.90, 0.87, 1.0))

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

    # ----- Tray hinge hardware on inner rear bowl wall -----
    tray_hinge_y = 0.050
    tray_hinge_z = 0.024
    for i, x in enumerate((-0.018, 0.018)):
        base.visual(Box((0.014, 0.010, 0.008)), origin=Origin(xyz=(x, tray_hinge_y, tray_hinge_z)), material=metal, name=f"tray_hinge_bracket_{i}")
        base.visual(Cylinder(radius=0.003, length=0.016), origin=Origin(xyz=(x, tray_hinge_y, tray_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)), material=metal, name=f"tray_hinge_barrel_{i}")

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

    # ----- Flip puff tray (child of base, sits above powder pan) -----
    puff_tray = model.part("puff_tray")
    # Part frame at the tray hinge origin; geometry extends forward (-Y) to bowl center.
    puff_tray.visual(
        mesh_from_cadquery(_puff_tray_disc(), "tray_disc"),
        origin=Origin(xyz=(0.0, -tray_hinge_y, -0.001)),
        material=tray_white,
        name="tray_disc",
    )
    # Applicator puff sponge seated onto the tray disc surface.
    puff_tray.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.0, -tray_hinge_y, 0.003)),
        material=sponge,
        name="applicator_puff",
    )
    # Tray hinge pins captured by the base brackets.
    for i, x in enumerate((-0.018, 0.018)):
        puff_tray.visual(
            Cylinder(radius=0.002, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"tray_hinge_pin_{i}",
        )

    model.articulation(
        "base_to_puff_tray",
        ArticulationType.REVOLUTE,
        parent=base,
        child=puff_tray,
        origin=Origin(xyz=(0.0, tray_hinge_y, tray_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.0, lower=0.0, upper=1.40),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    puff_tray = object_model.get_part("puff_tray")
    lid_hinge = object_model.get_articulation("base_to_lid")
    tray_hinge = object_model.get_articulation("base_to_puff_tray")

    # --- Lid hinge allowances ---
    for side in (0, 1):
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_barrel_{side}", reason="The lid hinge pin is captured by the rear hinge barrel.")
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_saddle_{side}", reason="The lid hinge pin passes through the hinge saddle bore.")
        ctx.allow_overlap(base, lid, elem_a=f"hinge_barrel_{side}", elem_b="white_domed_lid", reason="The rear hinge barrel is locally nested under the rear lid edge.")

    # --- Tray hinge allowances ---
    for i in (0, 1):
        ctx.allow_overlap(puff_tray, base, elem_a=f"tray_hinge_pin_{i}", elem_b=f"tray_hinge_barrel_{i}", reason="The tray hinge pin is captured by the tray hinge barrel on the inner bowl wall.")
        ctx.allow_overlap(base, puff_tray, elem_a=f"tray_hinge_bracket_{i}", elem_b="tray_disc", reason="The tray hinge bracket is locally nested under the rear tray edge.")

    # --- Lid checks ---
    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.080, name="closed lid covers the bowl footprint")
    ctx.expect_overlap(base, base, axes="xy", elem_a="powder_pan", elem_b="clear_bowl", min_overlap=0.080, name="powder pan seated inside the translucent bowl")
    ctx.expect_overlap(lid, lid, axes="xy", elem_a="inner_mirror", elem_b="white_domed_lid", min_overlap=0.090, name="mirror fills the inside of the lid")
    ctx.expect_overlap(lid, base, axes="x", elem_a="front_latch_tab", elem_b="front_latch_socket", min_overlap=0.020, name="front latch lines up with the socket")

    # --- Puff tray checks ---
    # Tray covers the powder pan when closed.
    ctx.expect_overlap(puff_tray, base, axes="xy", elem_a="tray_disc", elem_b="powder_pan", min_overlap=0.060, name="closed tray covers the powder pan")
    # Puff sits on the tray.
    ctx.expect_overlap(puff_tray, puff_tray, axes="xy", elem_a="applicator_puff", elem_b="tray_disc", min_overlap=0.050, name="applicator puff sits on the tray")
    # Tray sits above the powder pan in Z.
    ctx.expect_gap(puff_tray, base, axis="z", positive_elem="tray_disc", negative_elem="powder_dome", max_penetration=0.002, name="tray sits at or above the powder dome")

    # --- Lid opens upward ---
    closed_lid = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    with ctx.pose({lid_hinge: 1.20}):
        opened_lid = ctx.part_element_world_aabb(lid, elem="white_domed_lid")
    ctx.check("lid opens upward", closed_lid is not None and opened_lid is not None and opened_lid[1][2] > closed_lid[1][2] + 0.040, details=f"closed={closed_lid}, opened={opened_lid}")

    # --- Tray flips up to reveal powder ---
    closed_tray = ctx.part_element_world_aabb(puff_tray, elem="tray_disc")
    with ctx.pose({tray_hinge: 1.20}):
        flipped_tray = ctx.part_element_world_aabb(puff_tray, elem="tray_disc")
    ctx.check(
        "puff tray flips up to reveal powder",
        closed_tray is not None and flipped_tray is not None and flipped_tray[1][2] > closed_tray[1][2] + 0.020,
        details=f"closed={closed_tray}, flipped={flipped_tray}",
    )

    ctx.check(
        "compact has bowl, lid, tray, mirror, puff, and hinge layers",
        len(base.visuals) >= 10 and len(lid.visuals) >= 9 and len(puff_tray.visuals) >= 4,
        details=f"base={len(base.visuals)}, lid={len(lid.visuals)}, tray={len(puff_tray.visuals)}",
    )
    return ctx.report()


object_model = build_object_model()
