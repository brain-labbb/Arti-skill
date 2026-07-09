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
    """Rounded-square solid centered at origin."""
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(r)


def _hollow_base() -> cq.Workplane:
    """Glossy black tray with a recessed cavity that holds the powder pan."""
    body = _rounded_square(0.144, 0.134, 0.030, 0.022).translate((0.0, 0.0, 0.015))
    cavity = _rounded_square(0.120, 0.110, 0.022, 0.013).translate((0.0, 0.0, 0.022))
    return body.cut(cavity)


def _rail_profile() -> cq.Workplane:
    """A single sliding rail track: raised T-profile running along Y.

    Local origin at the bottom center of the rail. The rail is 0.005m wide,
    0.005m tall stem, with a 0.009m wide 0.002m tall cap on top. Total height
    0.007m above the base rim.
    """
    stem = cq.Workplane("XY").box(0.005, 0.118, 0.005).translate((0.0, 0.0, 0.0025))
    cap = cq.Workplane("XY").box(0.009, 0.118, 0.002).translate((0.0, 0.0, 0.006))
    return stem.union(cap)


def _slider_slot() -> cq.Workplane:
    """Lid slider block: an inverted U-channel that captures the base rail.

    Local origin at the bottom center. The channel is 0.011 wide (clearance over
    the 0.009 rail cap), 0.040 long (Y), and 0.008 tall with 0.002 thick walls.
    """
    outer = cq.Workplane("XY").box(0.013, 0.040, 0.008).translate((0.0, 0.0, 0.004))
    inner = cq.Workplane("XY").box(0.010, 0.042, 0.006).translate((0.0, 0.0, 0.005))
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_luxury_cushion_compact")

    black = _mat(model, "glossy_black", (0.012, 0.012, 0.012, 1.0))
    white = _mat(model, "glossy_white", (0.93, 0.92, 0.88, 1.0))
    chrome = _mat(model, "chrome_trim", (0.66, 0.67, 0.66, 1.0))
    logo_ink = _mat(model, "logo_black", (0.02, 0.02, 0.02, 1.0))
    mirror_mat = _mat(model, "mirror", (0.72, 0.78, 0.80, 0.55))
    powder = _mat(model, "beige_powder", (0.80, 0.64, 0.48, 1.0))

    # ===== Base tray (root) =====
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_hollow_base(), "compact_base"),
        material=black,
        name="black_tray",
    )

    # Beige powder cushion seated in the recess
    base.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=powder,
        name="powder_pan",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=powder,
        name="powder_dome",
    )

    # Front thumb-notch latch socket (catch for the sliding lid)
    base.visual(
        Box((0.030, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.068, 0.032)),
        material=chrome,
        name="front_latch_socket",
    )

    # Sliding rails on left and right rim edges (for-i loop for repeated parts)
    for i, x_pos in enumerate((-0.064, 0.064)):
        base.visual(
            mesh_from_cadquery(_rail_profile(), f"base_rail_{i}"),
            origin=Origin(xyz=(x_pos, 0.0, 0.030)),
            material=chrome,
            name=f"base_rail_{i}",
        )

    # Rail end stops at the front (-Y) to prevent lid from sliding off
    for i, x_pos in enumerate((-0.064, 0.064)):
        base.visual(
            Box((0.012, 0.006, 0.010)),
            origin=Origin(xyz=(x_pos, -0.062, 0.035)),
            material=chrome,
            name=f"rail_stop_{i}",
        )

    # ===== Sliding lid (child) =====
    # At q=0, the lid part frame coincides with the articulation frame at
    # the base top center. Lid visuals are expressed relative to this frame.
    lid = model.part("lid")

    # Black lid edge panel (slightly smaller than base to slide between rails)
    lid.visual(
        mesh_from_cadquery(_rounded_square(0.124, 0.130, 0.008, 0.020), "lid_black_edge"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=black,
        name="black_lid_edge",
    )

    # White glossy top panel
    lid.visual(
        mesh_from_cadquery(_rounded_square(0.108, 0.114, 0.005, 0.016), "lid_white_top"),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=white,
        name="white_lid_top",
    )

    # Chrome parting line (thin rod across the width)
    lid.visual(
        Cylinder(radius=0.0015, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, 0.0135), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="chrome_parting_line",
    )

    # Interlocking double-ring emblem (logo)
    for i, dx in enumerate((-0.011, 0.011)):
        ring = TorusGeometry(0.017, 0.0028, radial_segments=16, tubular_segments=28)
        lid.visual(
            mesh_from_geometry(ring, f"logo_ring_{i}"),
            origin=Origin(xyz=(dx, 0.0, 0.014)),
            material=logo_ink,
            name=f"logo_ring_{i}",
        )

    # Inner mirror on the underside of the lid (mount + reflective surface)
    lid.visual(
        Cylinder(radius=0.050, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0005)),
        material=chrome,
        name="mirror_mount",
    )
    lid.visual(
        Cylinder(radius=0.048, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=mirror_mat,
        name="inner_mirror",
    )

    # Front latch tab (engages with base socket when closed)
    lid.visual(
        Box((0.026, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.064, 0.004)),
        material=chrome,
        name="front_latch_tab",
    )

    # Slider blocks on left and right edges (capture the base rails)
    for i, x_pos in enumerate((-0.064, 0.064)):
        lid.visual(
            mesh_from_cadquery(_slider_slot(), f"lid_slider_{i}"),
            origin=Origin(xyz=(x_pos, 0.0, -0.001)),
            material=chrome,
            name=f"lid_slider_{i}",
        )

    # Prismatic articulation: lid slides along -Y (toward front) to expose pan
    # Origin at the base top surface center (the actual contact plane)
    model.articulation(
        "base_to_lid",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.3, lower=0.0, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    slide = object_model.get_articulation("base_to_lid")

    # Slider blocks intentionally capture the base rails (nested sliding fit)
    for i in (0, 1):
        ctx.allow_overlap(
            lid,
            base,
            elem_a=f"lid_slider_{i}",
            elem_b=f"base_rail_{i}",
            reason=f"The lid slider block {i} captures the base rail {i} for the prismatic sliding fit.",
        )

    # Closed-pose checks
    ctx.expect_overlap(
        lid, base, axes="xy", min_overlap=0.080,
        name="closed lid covers the base footprint",
    )
    ctx.expect_overlap(
        lid, lid, axes="xy", elem_a="white_lid_top", elem_b="black_lid_edge",
        min_overlap=0.090,
        name="white top sits within the black lid edge",
    )
    ctx.expect_gap(
        lid, base, axis="z", positive_elem="black_lid_edge", negative_elem="black_tray",
        min_gap=-0.002, max_gap=0.010,
        name="lid sits on top of the base with small clearance",
    )

    # Powder pan is hidden when closed (lid covers it)
    ctx.expect_overlap(
        lid, base, axes="xy", elem_a="black_lid_edge", elem_b="powder_pan",
        min_overlap=0.060,
        name="closed lid covers the powder pan",
    )

    # Open-pose: lid slides toward -Y to expose the powder pan
    with ctx.pose({slide: 0.075}):
        # Powder pan should be partially exposed (lid has slid forward)
        pan_aabb = ctx.part_element_world_aabb(base, elem="powder_pan")
        lid_aabb = ctx.part_element_world_aabb(lid, elem="black_lid_edge")
        if pan_aabb is not None and lid_aabb is not None:
            # The pan center Y should be behind (more positive Y) the lid rear edge
            pan_center_y = (pan_aabb[0][1] + pan_aabb[1][1]) / 2.0
            lid_rear_y = lid_aabb[0][1]  # min Y of lid (most forward)
            ctx.check(
                "sliding lid exposes powder pan",
                lid_aabb[1][1] < pan_center_y + 0.010,
                details=f"lid rear Y={lid_aabb[1][1]:.4f}, pan center Y={pan_center_y:.4f}",
            )
        else:
            ctx.fail("sliding lid exposes powder pan", "could not measure pan or lid AABB")

        # Lid still retains insertion on the rails (slider still overlaps rail in X)
        ctx.expect_overlap(
            lid, base, axes="x",
            min_overlap=0.080,
            name="open lid retains lateral engagement with base",
        )

    # Verify the lid actually translates in -Y when opened
    closed_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({slide: 0.075}):
        open_lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid slides forward along -Y",
        closed_lid_pos is not None
        and open_lid_pos is not None
        and open_lid_pos[1] < closed_lid_pos[1] - 0.040,
        details=f"closed={closed_lid_pos}, open={open_lid_pos}",
    )

    # Visual complexity: base has tray + pan + dome + socket + rails + stops
    # lid has edge + top + parting + 2 rings + mirror + tab + 2 sliders
    ctx.check(
        "compact has hollow tray, rails, logo and mirror",
        len(base.visuals) >= 7 and len(lid.visuals) >= 8,
        details=f"base visuals={len(base.visuals)}, lid visuals={len(lid.visuals)}",
    )

    return ctx.report()


object_model = build_object_model()
