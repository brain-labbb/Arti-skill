from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_ring(outer_w: float, outer_h: float, inner_w: float, inner_h: float, depth: float):
    """Rounded rectangular through-frame in local X/Z, extruded through Y."""
    return (
        cq.Workplane("XZ")
        .rect(outer_w, outer_h)
        .rect(inner_w, inner_h)
        .extrude(depth / 2.0, both=True)
        .edges("|Y")
        .fillet(0.018)
    )


def _rounded_panel(width: float, height: float, thickness: float):
    """Thin rounded rectangle in local X/Z, extruded through Y."""
    return (
        cq.Workplane("XZ")
        .rect(width, height)
        .extrude(thickness / 2.0, both=True)
        .edges("|Y")
        .fillet(0.018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cat_flap_panel",
        meta={
            "reference_note": (
                "Reference and category both indicate a cat flap / pet door; "
                "no classification mismatch suspected."
            )
        },
    )

    white_plastic = Material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    glass = Material("slightly_green_glass", rgba=(0.58, 0.78, 0.82, 0.30))
    smoke_polycarbonate = Material("smoky_translucent_flap", rgba=(0.36, 0.30, 0.24, 0.42))
    dark_rubber = Material("dark_rubber_gasket", rgba=(0.015, 0.014, 0.012, 1.0))
    screw_metal = Material("brushed_screw_heads", rgba=(0.55, 0.55, 0.52, 1.0))
    magnet = Material("black_magnet", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("panel_frame")

    # Minimal surrounding door/glass panel, cut away around the pet door.
    # Four overlapping strips keep the panel as one physically supported part.
    frame.visual(
        Box((0.19, 0.012, 0.58)),
        origin=Origin(xyz=(-0.285, 0.0, 0.18)),
        material=glass,
        name="panel_side_0",
    )
    frame.visual(
        Box((0.19, 0.012, 0.58)),
        origin=Origin(xyz=(0.285, 0.0, 0.18)),
        material=glass,
        name="panel_side_1",
    )
    frame.visual(
        Box((0.38, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=glass,
        name="panel_top",
    )
    frame.visual(
        Box((0.38, 0.012, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=glass,
        name="panel_bottom",
    )

    # White molded trim: a rounded rectangular ring with a real through-opening.
    frame.visual(
        mesh_from_cadquery(_rounded_ring(0.44, 0.42, 0.30, 0.32, 0.060), "molded_trim"),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=white_plastic,
        name="front_trim",
    )

    # Inner tunnel/sleeve surfaces visible through the opening.
    frame.visual(
        Box((0.018, 0.090, 0.315)),
        origin=Origin(xyz=(-0.159, 0.0, 0.18)),
        material=white_plastic,
        name="tunnel_side_0",
    )
    frame.visual(
        Box((0.018, 0.090, 0.315)),
        origin=Origin(xyz=(0.159, 0.0, 0.18)),
        material=white_plastic,
        name="tunnel_side_1",
    )
    frame.visual(
        Box((0.318, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.012, 0.349)),
        material=white_plastic,
        name="tunnel_top",
    )
    frame.visual(
        Box((0.318, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=white_plastic,
        name="tunnel_sill",
    )

    # Dark gasket strips and bottom magnet/latch visible at the entry.
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(-0.147, -0.034, 0.18)),
        material=dark_rubber,
        name="gasket_side_0",
    )
    frame.visual(
        Box((0.006, 0.006, 0.305)),
        origin=Origin(xyz=(0.147, -0.034, 0.18)),
        material=dark_rubber,
        name="gasket_side_1",
    )
    frame.visual(
        Box((0.255, 0.007, 0.006)),
        origin=Origin(xyz=(0.0, -0.0335, 0.337)),
        material=dark_rubber,
        name="top_gasket",
    )
    frame.visual(
        Box((0.070, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.038, 0.029)),
        material=magnet,
        name="bottom_magnet",
    )

    # Surface screws are shallow, seated into the plastic trim.
    screw_positions = [
        (-0.168, 0.334),
        (0.168, 0.334),
        (-0.168, 0.026),
        (0.168, 0.026),
        (-0.205, 0.180),
        (0.205, 0.180),
    ]
    for i, (x, z) in enumerate(screw_positions):
        frame.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, -0.032, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"screw_{i}",
        )
        frame.visual(
            Box((0.014, 0.002, 0.002)),
            origin=Origin(xyz=(x, -0.035, z), rpy=(0.0, 0.0, math.pi / 6.0)),
            material=dark_rubber,
            name=f"screw_slot_{i}",
        )

    # Fixed hinge knuckles at the sides, attached to the top trim.
    for i, x in enumerate((-0.165, 0.165)):
        frame.visual(
            Box((0.055, 0.028, 0.024)),
            origin=Origin(xyz=(x, -0.032, 0.340)),
            material=white_plastic,
            name=f"hinge_boss_{i}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=Origin(xyz=(x, -0.042, 0.348), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=white_plastic,
            name=f"fixed_hinge_pin_{i}",
        )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_rounded_panel(0.268, 0.282, 0.005), "translucent_swing_flap"),
        origin=Origin(xyz=(0.0, 0.006, -0.149)),
        material=smoke_polycarbonate,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.009, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="moving_hinge_barrel",
    )
    flap.visual(
        Box((0.090, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.003, -0.282)),
        material=screw_metal,
        name="magnet_keeper",
    )
    flap.visual(
        Box((0.252, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, -0.014)),
        material=dark_rubber,
        name="top_seal",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, -0.042, 0.348)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-1.05, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("panel_frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    flap_panel_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    if flap_panel_aabb is None:
        ctx.fail("flap panel has measurable bounds", "No AABB was returned for flap_panel.")
    else:
        lower, upper = flap_panel_aabb
        ctx.check(
            "flap panel fits inside pet-door opening",
            lower[0] > -0.150 and upper[0] < 0.150 and lower[2] > 0.020 and upper[2] < 0.342,
            details=f"flap_panel_aabb=({lower}, {upper})",
        )

    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="front_trim",
        negative_elem="flap_panel",
        name="closed flap sits just behind front trim",
    )

    rest_aabb = ctx.part_element_world_aabb(flap, elem="magnet_keeper")
    with ctx.pose({hinge: 0.75}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="magnet_keeper")
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) * 0.5
    open_center_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) * 0.5
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    open_center_z = None if open_aabb is None else (open_aabb[0][2] + open_aabb[1][2]) * 0.5
    ctx.check(
        "positive hinge swing opens flap outward and upward",
        rest_center_y is not None
        and open_center_y is not None
        and rest_center_z is not None
        and open_center_z is not None
        and open_center_y > rest_center_y + 0.12
        and open_center_z > rest_center_z + 0.035,
        details=f"rest={rest_aabb}, open={open_aabb}, open_flap={flap_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
