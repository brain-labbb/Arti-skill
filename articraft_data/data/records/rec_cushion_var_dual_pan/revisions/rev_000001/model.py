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


_PAN_X_SPAN = 0.060  # center-to-center distance between the two pans
_PAN_RADIUS = 0.025
_PAN_HEIGHT = 0.014
_DOME_RADIUS = 0.020
_DOME_HEIGHT = 0.003


def _pan_geometry() -> Cylinder:
    """Shared powder-pan cylinder used by both compartments."""
    return Cylinder(radius=_PAN_RADIUS, length=_PAN_HEIGHT)


def _dome_geometry() -> Cylinder:
    """Shared pressed-powder dome sitting on top of each pan."""
    return Cylinder(radius=_DOME_RADIUS, length=_DOME_HEIGHT)


def _hollow_base() -> cq.Workplane:
    """Glossy black tray with a recessed cavity that holds two powder pans."""
    body = _rounded_square(0.144, 0.134, 0.030, 0.022).translate((0.0, 0.0, 0.015))
    cavity = _rounded_square(0.120, 0.110, 0.020, 0.013).translate((0.0, 0.0, 0.023))
    return body.cut(cavity)


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
    # Two side-by-side beige powder pans seated in the recess.
    for i in range(2):
        x_offset = -_PAN_X_SPAN / 2.0 + i * _PAN_X_SPAN
        base.visual(_pan_geometry(), origin=Origin(xyz=(x_offset, 0.0, 0.020)), material=powder, name=f"powder_pan_{i}")
        base.visual(_dome_geometry(), origin=Origin(xyz=(x_offset, 0.0, 0.028)), material=powder, name=f"powder_dome_{i}")
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
    # Verify two side-by-side pans exist with proper spacing.
    pan_names = [f"powder_pan_{i}" for i in range(2)]
    dome_names = [f"powder_dome_{i}" for i in range(2)]
    for name in pan_names + dome_names:
        ctx.check(f"pan visual {name} exists", base.get_visual(name) is not None, details=f"missing {name} on base")

    ctx.expect_gap(
        base, base,
        axis="x",
        positive_elem="powder_pan_1",
        negative_elem="powder_pan_0",
        min_gap=0.002,
        name="two pans are separated along X",
    )
    ctx.expect_overlap(
        base, base,
        axes="y",
        elem_a="powder_pan_0",
        elem_b="powder_pan_1",
        min_overlap=0.030,
        name="pans share the same Y alignment",
    )

    ctx.check(
        "compact has hollow tray, two pans, logo and mirror",
        len(base.visuals) >= 8 and len(lid.visuals) >= 8,
        details="base should be a hollow tray with two powder pans and hinge; lid should include logo rings, mirror and latch",
    )
    return ctx.report()


object_model = build_object_model()
