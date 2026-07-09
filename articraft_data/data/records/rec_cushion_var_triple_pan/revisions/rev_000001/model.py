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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


def _rounded_square(sx: float, sy: float, sz: float, r: float) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(r)


def _hollow_base() -> cq.Workplane:
    """Glossy black tray with a recessed cavity that holds three powder pans."""
    body = _rounded_square(0.144, 0.134, 0.030, 0.022).translate((0.0, 0.0, 0.015))
    cavity = _rounded_square(0.120, 0.080, 0.020, 0.012).translate((0.0, 0.0, 0.023))
    return body.cut(cavity)


def _powder_pan_dish() -> cq.Workplane:
    """Single shallow powder pan: thin rim wall with a floor disk."""
    outer_r = 0.016
    inner_r = 0.014
    height = 0.010
    rim = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    floor = cq.Workplane("XY").circle(inner_r).extrude(0.001)
    return rim.union(floor)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_luxury_cushion_compact")

    black = _mat(model, "glossy_black", (0.012, 0.012, 0.012, 1.0))
    white = _mat(model, "glossy_white", (0.93, 0.92, 0.88, 1.0))
    chrome = _mat(model, "chrome_trim", (0.66, 0.67, 0.66, 1.0))
    logo_ink = _mat(model, "logo_black", (0.02, 0.02, 0.02, 1.0))
    mirror = _mat(model, "mirror", (0.72, 0.78, 0.80, 0.55))
    powder = _mat(model, "beige_powder", (0.80, 0.64, 0.48, 1.0))

    # ----- Base tray (root) -----
    base = model.part("base")
    base.visual(mesh_from_cadquery(_hollow_base(), "compact_base"), material=black, name="black_tray")
    # Three identical powder pans in a row, evenly spaced.
    _pan_spacing = 0.038
    for i in range(3):
        _px = (i - 1) * _pan_spacing
        base.visual(
            mesh_from_cadquery(_powder_pan_dish(), f"pan_dish_{i}"),
            origin=Origin(xyz=(_px, 0.0, 0.013)),
            material=chrome,
            name=f"pan_rim_{i}",
        )
        base.visual(
            Cylinder(radius=0.013, length=0.007),
            origin=Origin(xyz=(_px, 0.0, 0.0175)),
            material=powder,
            name=f"pan_powder_{i}",
        )
        base.visual(
            Cylinder(radius=0.011, length=0.002),
            origin=Origin(xyz=(_px, 0.0, 0.022)),
            material=powder,
            name=f"pan_dome_{i}",
        )
    # Front thumb-notch latch socket.
    base.visual(Box((0.034, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.066, 0.020)), material=chrome, name="front_latch_socket")
    # Rear hinge barrels.
    for side, x in enumerate((-0.032, 0.032)):
        base.visual(Cylinder(radius=0.005, length=0.030, ), origin=Origin(xyz=(x, 0.066, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name=f"hinge_barrel_{side}")

    # ----- Lid (child) authored in the rear-hinge pivot frame -----
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_rounded_square(0.144, 0.134, 0.012, 0.022), "lid_black_edge"), origin=Origin(xyz=(0.0, -0.066, 0.004)), material=black, name="black_lid_edge")
    lid.visual(mesh_from_cadquery(_rounded_square(0.128, 0.118, 0.008, 0.018), "lid_white_top"), origin=Origin(xyz=(0.0, -0.066, 0.012)), material=white, name="white_lid_top")
    lid.visual(Cylinder(radius=0.0015, length=0.140), origin=Origin(xyz=(0.0, -0.066, 0.0165), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="chrome_parting_line")
    # Interlocking double-ring emblem (reads as the double-C logo, not a cross).
    for side, dx in enumerate((-0.011, 0.011)):
        ring = TorusGeometry(0.017, 0.0028, radial_segments=16, tubular_segments=28)
        lid.visual(mesh_from_geometry(ring, f"logo_ring_{side}"), origin=Origin(xyz=(dx, -0.066, 0.018)), material=logo_ink, name=f"logo_ring_{side}")
    # Inner mirror on the underside of the lid.
    lid.visual(Cylinder(radius=0.050, length=0.0025), origin=Origin(xyz=(0.0, -0.066, -0.004)), material=mirror, name="inner_mirror")
    # Front latch tab.
    lid.visual(Box((0.030, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.133, 0.002)), material=chrome, name="front_latch_tab")
    # Short hinge pins seated into the rear barrels.
    for side, x in enumerate((-0.032, 0.032)):
        lid.visual(Cylinder(radius=0.0035, length=0.022), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name=f"lid_hinge_pin_{side}")

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.066, 0.032)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.2, velocity=1.4, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("base_to_lid")

    for side in (0, 1):
        ctx.allow_overlap(lid, base, elem_a=f"lid_hinge_pin_{side}", elem_b=f"hinge_barrel_{side}", reason="The lid hinge pin is captured by the rear hinge barrel.")
        ctx.allow_overlap(base, lid, elem_a=f"hinge_barrel_{side}", elem_b="black_lid_edge", reason="The rear hinge barrel is locally nested under the rear lid edge.")

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.090, name="closed lid covers the base footprint")
    ctx.expect_overlap(lid, lid, axes="xy", elem_a="white_lid_top", elem_b="black_lid_edge", min_overlap=0.100, name="white top sits on the black lid edge")
    ctx.expect_overlap(lid, base, axes="x", elem_a="front_latch_tab", elem_b="front_latch_socket", min_overlap=0.020, name="front latch lines up with the socket")

    closed = ctx.part_element_world_aabb(lid, elem="white_lid_top")
    with ctx.pose({hinge: 1.15}):
        opened = ctx.part_element_world_aabb(lid, elem="white_lid_top")
    ctx.check("lid opens upward", closed is not None and opened is not None and opened[1][2] > closed[1][2] + 0.035, details=f"closed={closed}, opened={opened}")
    # Three-pan layout checks
    pan_names = [f"pan_rim_{i}" for i in range(3)]
    for pn in pan_names:
        ctx.check(f"pan {pn} exists", base.get_visual(pn) is not None, details=f"missing {pn}")
    # Pans are evenly spaced along X
    pan_centers = []
    for pn in pan_names:
        v = base.get_visual(pn)
        if v is not None:
            pan_centers.append(v.origin.xyz[0] if v.origin else 0.0)
    if len(pan_centers) == 3:
        spacing_01 = abs(pan_centers[1] - pan_centers[0])
        spacing_12 = abs(pan_centers[2] - pan_centers[1])
        ctx.check(
            "pans are evenly spaced",
            abs(spacing_01 - spacing_12) < 0.002 and spacing_01 > 0.025,
            details=f"spacing_01={spacing_01:.4f}, spacing_12={spacing_12:.4f}",
        )
    # Each pan powder fill exists
    for i in range(3):
        ctx.check(f"pan_powder_{i} exists", base.get_visual(f"pan_powder_{i}") is not None, details=f"missing pan_powder_{i}")
    # Pans fit within the base cavity in XY
    for i in range(3):
        ctx.expect_within(base, base, axes="xy", elem_a=f"pan_rim_{i}", elem_b="black_tray", margin=0.001, name=f"pan_rim_{i} fits inside base tray")

    ctx.check("compact has hollow tray, three pans, logo and mirror", len(base.visuals) >= 13 and len(lid.visuals) >= 8, details="base should have tray + 3 pans (rim+powder+dome) + latch + 2 barrels; lid should include logo rings, mirror and latch")
    return ctx.report()


object_model = build_object_model()
