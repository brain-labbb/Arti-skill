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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TAU = 2.0 * math.pi


def _tube(z0: float, height: float, outer_r: float, inner_r: float) -> cq.Workplane:
    """Vertical annular tube authored in meters, from z0 to z0 + height."""
    outer = cq.Workplane("XY").workplane(offset=z0).circle(outer_r).extrude(height)
    cutter = cq.Workplane("XY").workplane(offset=z0 - 0.003).circle(inner_r).extrude(height + 0.006)
    return outer.cut(cutter)


def _ring_with_holes(
    z0: float,
    height: float,
    outer_r: float,
    inner_r: float,
    *,
    bolt_count: int,
    bolt_circle_r: float,
    hole_r: float,
) -> cq.Workplane:
    """Annular flange/plate with a central bore and repeated through holes."""
    body = _tube(z0, height, outer_r, inner_r)
    for i in range(bolt_count):
        a = TAU * i / bolt_count
        x = bolt_circle_r * math.cos(a)
        y = bolt_circle_r * math.sin(a)
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=z0 - 0.004)
            .center(x, y)
            .circle(hole_r)
            .extrude(height + 0.008)
        )
        body = body.cut(cutter)
    return body


def _circle_cylinder(z0: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(height)


def _add_radial_boxes(
    part,
    *,
    count: int,
    radius: float,
    z: float,
    size: tuple[float, float, float],
    material: Material,
    name_prefix: str,
    start_angle: float = 0.0,
) -> None:
    """Add repeated box features whose local +X points radially outward."""
    for i in range(count):
        a = start_angle + TAU * i / count
        part.visual(
            Box(size),
            origin=Origin(xyz=(radius * math.cos(a), radius * math.sin(a), z), rpy=(0.0, 0.0, a)),
            material=material,
            name=f"{name_prefix}_{i:02d}",
        )


def _add_radial_cylinders(
    part,
    *,
    count: int,
    radius: float,
    z: float,
    cyl_radius: float,
    cyl_height: float,
    material: Material,
    name_prefix: str,
    start_angle: float = 0.0,
) -> None:
    for i in range(count):
        a = start_angle + TAU * i / count
        part.visual(
            Cylinder(radius=cyl_radius, length=cyl_height),
            origin=Origin(xyz=(radius * math.cos(a), radius * math.sin(a), z)),
            material=material,
            name=f"{name_prefix}_{i:02d}",
        )


def _add_bearing_balls(
    part,
    *,
    count: int,
    radius: float,
    z: float,
    ball_radius: float,
    material: Material,
    name_prefix: str,
    start_angle: float = 0.0,
) -> None:
    for i in range(count):
        a = start_angle + TAU * i / count
        part.visual(
            Sphere(radius=ball_radius),
            origin=Origin(xyz=(radius * math.cos(a), radius * math.sin(a), z)),
            material=material,
            name=f"{name_prefix}_{i:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harmonic_drive_reducer")

    satin = model.material("satin_silver", rgba=(0.72, 0.73, 0.72, 1.0))
    dark_satin = model.material("dark_satin", rgba=(0.20, 0.21, 0.21, 1.0))
    black = model.material("black_oxide", rgba=(0.004, 0.004, 0.005, 1.0))
    bearing = model.material("polished_bearing_steel", rgba=(0.90, 0.91, 0.88, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(
            _ring_with_holes(
                0.000,
                0.018,
                0.160,
                0.074,
                bolt_count=24,
                bolt_circle_r=0.132,
                hole_r=0.0085,
            ),
            "wide_bolted_flange",
            tolerance=0.0008,
            angular_tolerance=0.04,
        ),
        material=satin,
        name="wide_bolted_flange",
    )
    housing.visual(
        mesh_from_cadquery(_tube(0.012, 0.120, 0.108, 0.074), "ribbed_lower_shell", tolerance=0.0008),
        material=satin,
        name="ribbed_lower_shell",
    )
    housing.visual(
        mesh_from_cadquery(_tube(0.120, 0.014, 0.110, 0.077), "top_lip", tolerance=0.0008),
        material=satin,
        name="top_lip",
    )
    housing.visual(
        mesh_from_cadquery(_tube(0.024, 0.092, 0.074, 0.066), "flexspline_tooth_band", tolerance=0.0008),
        material=dark_satin,
        name="flexspline_tooth_band",
    )
    _add_radial_boxes(
        housing,
        count=60,
        radius=0.111,
        z=0.073,
        size=(0.008, 0.0038, 0.098),
        material=satin,
        name_prefix="outer_rib",
    )
    _add_radial_boxes(
        housing,
        count=72,
        radius=0.069,
        z=0.071,
        size=(0.009, 0.0022, 0.084),
        material=dark_satin,
        name_prefix="inner_tooth",
        start_angle=TAU / 144.0,
    )
    cartridge = model.part("upper_cartridge")
    cartridge.visual(
        mesh_from_cadquery(_tube(-0.064, 0.064, 0.062, 0.056), "inserted_cartridge_sleeve", tolerance=0.0008),
        material=dark_satin,
        name="inserted_cartridge_sleeve",
    )
    cartridge.visual(
        mesh_from_cadquery(
            _ring_with_holes(
                0.000,
                0.014,
                0.108,
                0.030,
                bolt_count=20,
                bolt_circle_r=0.083,
                hole_r=0.0042,
            ),
            "lower_cover_plate",
            tolerance=0.0008,
            angular_tolerance=0.04,
        ),
        material=satin,
        name="lower_cover_plate",
    )
    cartridge.visual(
        mesh_from_cadquery(_tube(0.014, 0.017, 0.092, 0.031), "raised_middle_plate", tolerance=0.0008),
        material=satin,
        name="raised_middle_plate",
    )
    cartridge.visual(
        mesh_from_cadquery(_tube(0.031, 0.014, 0.072, 0.029), "small_top_plate", tolerance=0.0008),
        material=satin,
        name="small_top_plate",
    )
    cartridge.visual(
        mesh_from_cadquery(_tube(0.031, 0.006, 0.096, 0.088), "outer_black_retainer", tolerance=0.0008),
        material=black,
        name="outer_black_retainer",
    )
    cartridge.visual(
        mesh_from_cadquery(_tube(0.044, 0.007, 0.045, 0.027), "central_black_retainer", tolerance=0.0008),
        material=black,
        name="central_black_retainer",
    )
    _add_radial_cylinders(
        cartridge,
        count=20,
        radius=0.083,
        z=0.016,
        cyl_radius=0.0046,
        cyl_height=0.004,
        material=black,
        name_prefix="cover_socket",
        start_angle=TAU / 40.0,
    )
    _add_bearing_balls(
        cartridge,
        count=28,
        radius=0.083,
        z=0.035,
        ball_radius=0.0042,
        material=bearing,
        name_prefix="bearing_ball",
        start_angle=TAU / 56.0,
    )

    input_shaft = model.part("input_shaft")
    input_shaft.visual(
        Cylinder(radius=0.014, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=black,
        name="input_stem",
    )
    input_shaft.visual(
        Cylinder(radius=0.017, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=black,
        name="tall_input_shaft",
    )
    input_shaft.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=black,
        name="shaft_lower_collar",
    )
    input_shaft.visual(
        Cylinder(radius=0.041, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0545)),
        material=black,
        name="shaft_thrust_flange",
    )
    input_shaft.visual(
        mesh_from_cadquery(_circle_cylinder(-0.055, 0.029, 0.024), "wave_cam", tolerance=0.0008),
        material=dark_satin,
        name="wave_cam",
    )
    input_shaft.visual(
        mesh_from_cadquery(_circle_cylinder(-0.048, 0.008, 0.021), "wave_generator_face", tolerance=0.0008),
        material=bearing,
        name="wave_generator_face",
    )
    model.articulation(
        "cartridge_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "input_shaft_slide",
        ArticulationType.PRISMATIC,
        parent=cartridge,
        child=input_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.16, lower=0.0, upper=0.115),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    cartridge = object_model.get_part("upper_cartridge")
    input_shaft = object_model.get_part("input_shaft")
    slide = object_model.get_articulation("cartridge_slide")
    shaft_slide = object_model.get_articulation("input_shaft_slide")

    ctx.expect_contact(
        cartridge,
        housing,
        elem_a="lower_cover_plate",
        elem_b="top_lip",
        contact_tol=0.0015,
        name="seated cartridge cover rests on housing lip",
    )
    ctx.expect_within(
        input_shaft,
        cartridge,
        axes="xy",
        inner_elem="wave_cam",
        outer_elem="inserted_cartridge_sleeve",
        margin=0.001,
        name="wave generator fits inside cartridge sleeve",
    )
    ctx.expect_within(
        input_shaft,
        housing,
        axes="xy",
        inner_elem="wave_cam",
        outer_elem="flexspline_tooth_band",
        margin=0.001,
        name="wave generator fits inside flexspline bore",
    )
    wave_cam_aabb = ctx.part_element_world_aabb(input_shaft, elem="wave_cam")
    if wave_cam_aabb is not None:
        lo, hi = wave_cam_aabb
        wave_cam_diameter = max(float(hi[0] - lo[0]), float(hi[1] - lo[1]))
        ctx.check(
            "round wave cam stays smaller than black center ring mouth",
            wave_cam_diameter <= 0.0535,
            details=f"diameter={wave_cam_diameter}",
        )

    cartridge_rest_pos = ctx.part_world_position(cartridge)
    shaft_rest_pos = ctx.part_world_position(input_shaft)
    with ctx.pose({slide: 0.180}):
        cartridge_lifted_pos = ctx.part_world_position(cartridge)
        ctx.expect_gap(
            cartridge,
            housing,
            axis="z",
            min_gap=0.055,
            positive_elem="inserted_cartridge_sleeve",
            negative_elem="top_lip",
            name="exploded slide clears cartridge above housing",
        )
        ctx.expect_gap(
            input_shaft,
            housing,
            axis="z",
            min_gap=0.045,
            positive_elem="wave_cam",
            negative_elem="top_lip",
            name="exploded view exposes lifted wave generator",
        )

    ctx.check(
        "upper cartridge slides upward",
        cartridge_rest_pos is not None
        and cartridge_lifted_pos is not None
        and cartridge_lifted_pos[2] > cartridge_rest_pos[2] + 0.170,
        details=f"rest={cartridge_rest_pos}, lifted={cartridge_lifted_pos}",
    )

    with ctx.pose({shaft_slide: 0.115}):
        shaft_lifted_pos = ctx.part_world_position(input_shaft)
        ctx.expect_gap(
            input_shaft,
            cartridge,
            axis="z",
            min_gap=0.050,
            positive_elem="shaft_lower_collar",
            negative_elem="central_black_retainer",
            name="input shaft slides upward out of cartridge",
        )
        ctx.expect_gap(
            input_shaft,
            cartridge,
            axis="z",
            min_gap=0.045,
            positive_elem="wave_cam",
            negative_elem="inserted_cartridge_sleeve",
            name="second slide exposes wave cam above cartridge sleeve",
        )

    ctx.check(
        "input shaft is second sliding component",
        shaft_rest_pos is not None
        and shaft_lifted_pos is not None
        and shaft_lifted_pos[2] > shaft_rest_pos[2] + 0.105,
        details=f"rest={shaft_rest_pos}, lifted={shaft_lifted_pos}",
    )

    return ctx.report()


object_model = build_object_model()
