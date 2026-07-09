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


BASE_X = 0.180
BASE_Y = 0.120
PLATE_THICKNESS = 0.008
FOOT_HEIGHT = 0.025
SOCKET_HEIGHT = 0.024
LOWER_TUBE_LENGTH = 0.520
PRISMATIC_Z = 0.500
INNER_TUBE_LENGTH = 0.470
INNER_TUBE_BOTTOM = -0.180
INNER_TUBE_TOP = INNER_TUBE_BOTTOM + INNER_TUBE_LENGTH
SHAFT_TRAVEL = 0.120


def _tube_shape(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    z0: float = 0.0,
) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner).translate((0.0, 0.0, z0))


def _base_plate_shape() -> cq.Workplane:
    plate_z = FOOT_HEIGHT + PLATE_THICKNESS / 2.0
    plate = (
        cq.Workplane("XY")
        .box(BASE_X, BASE_Y, PLATE_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, plate_z))
    )
    socket_bottom = FOOT_HEIGHT + PLATE_THICKNESS - 0.001
    socket_outer = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(SOCKET_HEIGHT)
        .translate((0.0, 0.0, socket_bottom))
    )
    socket_bore = (
        cq.Workplane("XY")
        .circle(0.015)
        .extrude(SOCKET_HEIGHT + 0.006)
        .translate((0.0, 0.0, socket_bottom - 0.003))
    )
    return plate.union(socket_outer.cut(socket_bore))


def _collar_ring_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.023).extrude(0.036)
    bore = (
        cq.Workplane("XY")
        .circle(0.0140)
        .extrude(0.044)
        .translate((0.0, 0.0, -0.004))
    )
    ring = outer.cut(bore).translate((0.0, 0.0, -0.018))
    split_slot = cq.Workplane("XY").box(0.010, 0.060, 0.046).translate((0.0, -0.024, 0.0))
    return ring.cut(split_slot)


def _handle_shape() -> cq.Workplane:
    grip = (
        cq.Workplane("XY")
        .box(0.155, 0.042, 0.036)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z or <Z")
        .fillet(0.006)
        .translate((0.0, 0.0, 0.070))
    )
    neck = cq.Workplane("XY").circle(0.017).extrude(0.060)
    return grip.union(neck)


def _add_foot_visuals(foot, rubber: Material) -> None:
    foot.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=rubber,
        name="upper_plug",
    )
    foot.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=rubber,
        name="rubber_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quad_cane")

    chrome = model.material("brushed_chrome", rgba=(0.78, 0.80, 0.78, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.02, 0.022, 1.0))
    hole_shadow = model.material("hole_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate_socket"),
        material=dark_metal,
        name="plate_socket",
    )

    foot_positions = (
        (-0.070, -0.045),
        (0.070, -0.045),
        (-0.070, 0.045),
        (0.070, 0.045),
    )
    for index, (x, y) in enumerate(foot_positions):
        foot = model.part(f"foot_{index}")
        _add_foot_visuals(foot, black_rubber)
        model.articulation(
            f"base_to_foot_{index}",
            ArticulationType.FIXED,
            parent=base_plate,
            child=foot,
            origin=Origin(xyz=(x, y, FOOT_HEIGHT)),
        )

    lower_shaft = model.part("lower_shaft")
    lower_shaft.visual(
        mesh_from_cadquery(_tube_shape(0.0140, 0.0116, LOWER_TUBE_LENGTH), "lower_outer_tube"),
        material=chrome,
        name="outer_tube",
    )
    lower_shaft.visual(
        Cylinder(radius=0.0144, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=satin_black,
        name="base_bushing",
    )
    lower_shaft.visual(
        Cylinder(radius=0.0200, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=chrome,
        name="mounting_flange",
    )
    # Dark hole marks on the front of the lower tube make the telescoping height adjustment legible.
    for index, z in enumerate((0.145, 0.190, 0.235, 0.280, 0.325, 0.370, 0.415)):
        lower_shaft.visual(
            Cylinder(radius=0.0032, length=0.0016),
            origin=Origin(xyz=(0.0, -0.0143, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hole_shadow,
            name=f"adjustment_hole_{index}",
        )

    model.articulation(
        "base_to_lower_shaft",
        ArticulationType.FIXED,
        parent=base_plate,
        child=lower_shaft,
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + PLATE_THICKNESS + SOCKET_HEIGHT - 0.001)),
    )

    collar = model.part("height_collar")
    for name, size, xyz in (
        ("collar_side_0", (0.010, 0.044, 0.036), (0.019, 0.0, 0.0)),
        ("collar_side_1", (0.010, 0.044, 0.036), (-0.019, 0.0, 0.0)),
        ("collar_front", (0.038, 0.010, 0.036), (0.0, 0.019, 0.0)),
        ("collar_rear", (0.030, 0.010, 0.036), (0.0, -0.019, 0.0)),
    ):
        collar.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=chrome,
            name=name,
        )
    collar.visual(
        Cylinder(radius=0.0040, length=0.032),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="thumb_screw",
    )
    collar.visual(
        Cylinder(radius=0.0115, length=0.014),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="thumb_knob",
    )
    model.articulation(
        "lower_shaft_to_collar",
        ArticulationType.FIXED,
        parent=lower_shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_Z)),
    )

    upper_shaft = model.part("upper_shaft")
    upper_shaft.visual(
        Cylinder(radius=0.0095, length=INNER_TUBE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_BOTTOM + INNER_TUBE_LENGTH / 2.0)),
        material=chrome,
        name="inner_tube",
    )
    upper_shaft.visual(
        Cylinder(radius=0.0110, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_TOP - 0.007)),
        material=chrome,
        name="top_ferrule",
    )
    upper_shaft.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.0, -0.0106, -0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="spring_button",
    )
    model.articulation(
        "lower_shaft_to_upper_shaft",
        ArticulationType.PRISMATIC,
        parent=lower_shaft,
        child=upper_shaft,
        origin=Origin(xyz=(0.0, 0.0, PRISMATIC_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=SHAFT_TRAVEL),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "ergonomic_t_handle"),
        material=satin_black,
        name="t_grip",
    )
    handle.visual(
        Cylinder(radius=0.0098, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=chrome,
        name="shaft_insert",
    )
    model.articulation(
        "upper_shaft_to_handle",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_TOP)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_plate = object_model.get_part("base_plate")
    lower_shaft = object_model.get_part("lower_shaft")
    upper_shaft = object_model.get_part("upper_shaft")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("lower_shaft_to_upper_shaft")

    foot_names = [f"foot_{index}" for index in range(4)]
    ctx.check(
        "four named rubber feet",
        all(object_model.get_part(name) is not None for name in foot_names),
        details=f"expected feet {foot_names}",
    )

    for name in foot_names:
        foot = object_model.get_part(name)
        ctx.expect_gap(
            base_plate,
            foot,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{name} touches underside of plate",
        )
        ctx.expect_overlap(
            base_plate,
            foot,
            axes="xy",
            min_overlap=0.015,
            name=f"{name} sits under plate footprint",
        )

    ctx.expect_within(
        upper_shaft,
        lower_shaft,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.002,
        name="inner tube is centered inside lower tube",
    )
    ctx.expect_overlap(
        upper_shaft,
        lower_shaft,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.150,
        name="collapsed shaft keeps retained insertion",
    )
    ctx.expect_contact(
        handle,
        upper_shaft,
        elem_a="shaft_insert",
        elem_b="inner_tube",
        contact_tol=0.001,
        name="handle seats on top of telescoping tube",
    )

    rest_handle = ctx.part_world_aabb(handle)
    with ctx.pose({slide: SHAFT_TRAVEL}):
        ctx.expect_within(
            upper_shaft,
            lower_shaft,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.002,
            name="extended inner tube remains centered",
        )
        ctx.expect_overlap(
            upper_shaft,
            lower_shaft,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.040,
            name="extended shaft keeps retained insertion",
        )
        extended_handle = ctx.part_world_aabb(handle)

    ctx.check(
        "prismatic adjustment raises handle",
        rest_handle is not None
        and extended_handle is not None
        and extended_handle[1][2] > rest_handle[1][2] + SHAFT_TRAVEL * 0.8,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    return ctx.report()


object_model = build_object_model()
