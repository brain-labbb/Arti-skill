from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


AXIS_TO_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _axis_x_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    """Place a local-Z axis primitive/lathe so its axis runs along world X."""

    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yellow_handheld_flashlight")

    yellow = model.material("glossy_yellow_plastic", rgba=(1.0, 0.86, 0.0, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    dark_button = model.material("dark_button_insert", rgba=(0.02, 0.022, 0.02, 1.0))
    silver = model.material("mirror_silver_reflector", rgba=(0.86, 0.88, 0.86, 1.0))
    lens_mat = model.material("clear_polycarbonate", rgba=(0.82, 0.94, 1.0, 0.35))
    led_mat = model.material("warm_led", rgba=(1.0, 0.86, 0.45, 1.0))

    body = model.part("body")

    # Cylindrical battery barrel, about a three-cell handheld torch scale.
    body.visual(
        Cylinder(radius=0.028, length=0.180),
        origin=_axis_x_origin(0.055),
        material=yellow,
        name="barrel_shell",
    )

    # Tapered shoulder from the battery barrel to the wider head.
    shoulder = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.029, -0.034),
            (0.036, -0.050),
            (0.046, -0.073),
        ],
        inner_profile=[
            (0.024, -0.032),
            (0.030, -0.050),
            (0.039, -0.073),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(shoulder, "shoulder_shell"),
        origin=AXIS_TO_X,
        material=yellow,
        name="shoulder_shell",
    )

    # Hollow wider bezel head so the reflector and clear lens are actually visible.
    head_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.052, -0.158),
            (0.053, -0.145),
            (0.052, -0.082),
            (0.048, -0.073),
        ],
        inner_profile=[
            (0.041, -0.158),
            (0.041, -0.145),
            (0.040, -0.086),
            (0.035, -0.073),
        ],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )
    body.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        origin=AXIS_TO_X,
        material=yellow,
        name="head_shell",
    )

    # Black ring that rims the head at the lens opening, plus the rear black band.
    front_ring = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0535, -0.161), (0.0535, -0.154)],
        inner_profile=[(0.0415, -0.161), (0.0415, -0.154)],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(front_ring, "front_bezel_ring"),
        origin=AXIS_TO_X,
        material=black,
        name="front_bezel_ring",
    )

    rear_band = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.0525, -0.082), (0.0525, -0.068)],
        inner_profile=[(0.0435, -0.082), (0.0435, -0.068)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    body.visual(
        mesh_from_geometry(rear_band, "rear_bezel_band"),
        origin=AXIS_TO_X,
        material=black,
        name="rear_bezel_band",
    )

    # Raised longitudinal ribs on the bezel head.
    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        y = 0.055 * math.cos(theta)
        z = 0.055 * math.sin(theta)
        body.visual(
            Cylinder(radius=0.0030, length=0.052),
            origin=_axis_x_origin(-0.118, y, z),
            material=yellow,
            name=f"head_rib_{i}",
        )

    # Subtle grip rails along the barrel, as molded plastic rather than separate floating strips.
    for i, theta in enumerate((math.pi / 2.0, 3.0 * math.pi / 2.0, 0.0, math.pi)):
        y = 0.0295 * math.cos(theta)
        z = 0.0295 * math.sin(theta)
        body.visual(
            Cylinder(radius=0.0018, length=0.128),
            origin=_axis_x_origin(0.055, y, z),
            material=yellow,
            name=f"barrel_grip_{i}",
        )

    # Black tail cap/ring and a small lanyard eyelet on the tail end.
    body.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=_axis_x_origin(0.153),
        material=black,
        name="tail_cap",
    )
    body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.020, tube=0.0026, radial_segments=18, tubular_segments=56),
            "tail_eyelet",
        ),
        origin=_axis_x_origin(0.162),
        material=black,
        name="tail_eyelet",
    )

    # Reflector and lens are fixed optical subfeatures nested in the hollow head.
    reflector_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0415, -0.153),
            (0.030, -0.143),
            (0.019, -0.126),
            (0.009, -0.108),
        ],
        inner_profile=[
            (0.0385, -0.151),
            (0.027, -0.141),
            (0.016, -0.125),
            (0.006, -0.110),
        ],
        segments=112,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    body.visual(
        mesh_from_geometry(reflector_shell, "parabolic_reflector"),
        origin=AXIS_TO_X,
        material=silver,
        name="parabolic_reflector",
    )
    body.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.108, 0.0, 0.0)),
        material=led_mat,
        name="led_bulb",
    )

    body.visual(
        Cylinder(radius=0.0415, length=0.004),
        origin=_axis_x_origin(-0.158),
        material=lens_mat,
        name="lens_disc",
    )

    # Black oval side push-button; child frame sits at the barrel surface and
    # positive joint motion presses inward toward the flashlight axis.
    button = model.part("button")
    button_base = ExtrudeGeometry(
        rounded_rect_profile(0.038, 0.024, radius=0.012, corner_segments=10),
        0.006,
        cap=True,
        center=True,
    )
    button.visual(
        mesh_from_geometry(button_base, "button_base"),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="button_base",
    )
    button_cap = ExtrudeGeometry(
        rounded_rect_profile(0.026, 0.016, radius=0.008, corner_segments=10),
        0.0022,
        cap=True,
        center=True,
    )
    button.visual(
        mesh_from_geometry(button_cap, "button_cap"),
        origin=Origin(xyz=(0.0, 0.0071, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_button,
        name="button_cap",
    )

    # Nylon wrist strap: one continuous flexible loop emerging from the tail eyelet.
    strap = model.part("strap")
    strap_loop = tube_from_spline_points(
        [
            (0.004, 0.018, 0.000),
            (0.032, 0.038, -0.002),
            (0.090, 0.044, -0.006),
            (0.124, 0.000, -0.008),
            (0.092, -0.044, -0.006),
            (0.030, -0.036, -0.002),
            (0.004, -0.018, 0.000),
        ],
        radius=0.0024,
        samples_per_segment=18,
        closed_spline=True,
        radial_segments=18,
        cap_ends=False,
    )
    strap.visual(
        mesh_from_geometry(strap_loop, "strap_loop"),
        origin=Origin(),
        material=black,
        name="strap_loop",
    )

    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(-0.008, 0.028, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.004),
    )
    model.articulation(
        "body_to_strap",
        ArticulationType.FIXED,
        parent=body,
        child=strap,
        origin=Origin(xyz=(0.162, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("button")
    strap = object_model.get_part("strap")
    switch = object_model.get_articulation("body_to_button")

    ctx.allow_overlap(
        strap,
        body,
        elem_a="strap_loop",
        elem_b="tail_eyelet",
        reason="The soft nylon strap is threaded through the tail eyelet opening.",
    )

    barrel_box = ctx.part_element_world_aabb(body, elem="barrel_shell")
    head_box = ctx.part_element_world_aabb(body, elem="head_shell")
    ctx.check(
        "wider head than barrel",
        barrel_box is not None
        and head_box is not None
        and (head_box[1][1] - head_box[0][1]) > (barrel_box[1][1] - barrel_box[0][1]) + 0.035,
        details=f"barrel={barrel_box}, head={head_box}",
    )

    lens_box = ctx.part_element_world_aabb(body, elem="lens_disc")
    front_ring_box = ctx.part_element_world_aabb(body, elem="front_bezel_ring")
    reflector_box = ctx.part_element_world_aabb(body, elem="parabolic_reflector")
    ctx.check(
        "clear lens seated inside black front bezel",
        lens_box is not None
        and front_ring_box is not None
        and lens_box[0][1] >= front_ring_box[0][1] - 0.0015
        and lens_box[1][1] <= front_ring_box[1][1] + 0.0015
        and lens_box[0][2] >= front_ring_box[0][2] - 0.0015
        and lens_box[1][2] <= front_ring_box[1][2] + 0.0015,
        details=f"lens={lens_box}, ring={front_ring_box}",
    )
    ctx.check(
        "reflector visible behind lens",
        reflector_box is not None
        and lens_box is not None
        and min(reflector_box[1][1], lens_box[1][1]) - max(reflector_box[0][1], lens_box[0][1]) > 0.030
        and min(reflector_box[1][2], lens_box[1][2]) - max(reflector_box[0][2], lens_box[0][2]) > 0.030,
        details=f"reflector={reflector_box}, lens={lens_box}",
    )
    ctx.check(
        "lens sits just in front of reflector",
        reflector_box is not None
        and lens_box is not None
        and 0.0 <= reflector_box[0][0] - lens_box[1][0] <= 0.006,
        details=f"reflector={reflector_box}, lens={lens_box}",
    )

    ctx.expect_gap(
        button,
        body,
        axis="y",
        positive_elem="button_base",
        negative_elem="barrel_shell",
        min_gap=0.0,
        max_gap=0.0015,
        name="side button rests on barrel surface",
    )
    rest_pos = ctx.part_world_position(button)
    with ctx.pose({switch: 0.004}):
        pressed_pos = ctx.part_world_position(button)
    ctx.check(
        "button presses inward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] < rest_pos[1] - 0.003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    ctx.expect_gap(
        strap,
        body,
        axis="x",
        positive_elem="strap_loop",
        negative_elem="tail_eyelet",
        min_gap=-0.006,
        max_gap=0.010,
        name="strap loop emerges from tail eyelet",
    )
    ctx.expect_overlap(
        strap,
        body,
        axes="yz",
        elem_a="strap_loop",
        elem_b="tail_eyelet",
        min_overlap=0.010,
        name="strap loop is aligned with tail eyelet",
    )

    return ctx.report()


object_model = build_object_model()
