from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _torus_y(major_radius: float, tube_radius: float, *, major_segments: int = 96, tube_segments: int = 14) -> MeshGeometry:
    """Build a round-section iron hoop in the local XZ plane, spinning about local Y."""
    geom = MeshGeometry()
    for i in range(major_segments):
        theta = 2.0 * math.pi * i / major_segments
        radial = (math.cos(theta), 0.0, math.sin(theta))
        for j in range(tube_segments):
            phi = 2.0 * math.pi * j / tube_segments
            r = major_radius + tube_radius * math.cos(phi)
            x = r * radial[0]
            y = tube_radius * math.sin(phi)
            z = r * radial[2]
            geom.add_vertex(x, y, z)

    for i in range(major_segments):
        ni = (i + 1) % major_segments
        for j in range(tube_segments):
            nj = (j + 1) % tube_segments
            a = i * tube_segments + j
            b = ni * tube_segments + j
            c = ni * tube_segments + nj
            d = i * tube_segments + nj
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _straight_tube(points, radius: float, *, radial_segments: int = 14) -> MeshGeometry:
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=8,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def _wheel_lug(angle: float) -> MeshGeometry:
    """A short curved hoe lug welded to the wheel rim."""
    c = math.cos(angle)
    s = math.sin(angle)
    tangent = (-s, 0.0, c)
    radial = (c, 0.0, s)
    start = (0.302 * radial[0], 0.0, 0.302 * radial[2])
    mid = (
        0.328 * radial[0] + 0.014 * tangent[0],
        0.0,
        0.328 * radial[2] + 0.014 * tangent[2],
    )
    tip = (
        0.346 * radial[0] + 0.026 * tangent[0],
        0.0,
        0.346 * radial[2] + 0.026 * tangent[2],
    )
    return _straight_tube([start, mid, tip], 0.0052, radial_segments=10)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_hand_cultivator",
        meta={
            "category": "Agricultural",
            "small_class": "Hand cultivator",
            "description": "Walk-behind hand cultivator with twin wooden handles, rotating tine wheel, and rear spring claws.",
        },
    )

    wood = Material("pale_oiled_wood", rgba=(0.78, 0.64, 0.42, 1.0))
    grip = Material("black_rubber_grip", rgba=(0.035, 0.038, 0.035, 1.0))
    rust = Material("weathered_rusted_steel", rgba=(0.53, 0.26, 0.13, 1.0))
    dark = Material("dark_bolt_hardware", rgba=(0.055, 0.050, 0.045, 1.0))
    spring = Material("blue_black_spring_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    worn = Material("worn_bright_edge", rgba=(0.72, 0.70, 0.64, 1.0))

    frame = model.part("frame")

    # Twin long handles: pale wood with short rubberized hand-contact sleeves.
    for side, y in (("0", -0.18), ("1", 0.18)):
        lower_y = -0.082 if y < 0 else 0.082
        handle_points = [
            (-0.23, lower_y, 0.50),
            (-0.55, y * 0.82, 0.78),
            (-0.92, y * 0.94, 1.03),
            (-1.42, y, 1.25),
        ]
        frame.visual(
            mesh_from_geometry(_straight_tube(handle_points, 0.021, radial_segments=18), f"wood_handle_{side}"),
            material=wood,
            name=f"wood_handle_{side}",
        )
        frame.visual(
            mesh_from_geometry(_straight_tube(handle_points[-2:], 0.024, radial_segments=18), f"rubber_grip_{side}"),
            material=grip,
            name=f"rubber_grip_{side}",
        )

    # Rusted steel fork, braces, and rear head mount. The paired side rails sit
    # outboard of the wheel plane so only the axle passes through the rotating hub.
    for side, y in (("0", -0.074), ("1", 0.074)):
        frame.visual(
            mesh_from_geometry(
                _straight_tube([(0.0, y, 0.33), (-0.34, y, 0.37), (-0.72, y * 0.72, 0.18)], 0.0105),
                f"side_rail_{side}",
            ),
            material=rust,
            name=f"side_rail_{side}",
        )
        frame.visual(
            mesh_from_geometry(
                _straight_tube([(-0.17, y, 0.42), (-0.32, y * 1.18, 0.60), (-0.50, y * 1.48, 0.82)], 0.0085),
                f"upright_strap_{side}",
            ),
            material=rust,
            name=f"upright_strap_{side}",
        )
        frame.visual(
            mesh_from_geometry(
                _straight_tube([(-0.02, y, 0.33), (-0.36, y * 1.02, 0.52), (-0.60, y * 1.18, 0.63)], 0.0065),
                f"diagonal_brace_{side}",
            ),
            material=rust,
            name=f"diagonal_brace_{side}",
        )

    frame.visual(
        mesh_from_geometry(_straight_tube([(-0.36, -0.105, 0.62), (-0.36, 0.105, 0.62)], 0.012), "handle_clamp_bar"),
        material=rust,
        name="handle_clamp_bar",
    )
    frame.visual(
        mesh_from_geometry(_straight_tube([(-0.70, -0.075, 0.18), (-0.70, 0.075, 0.18)], 0.013), "head_crossbar"),
        material=rust,
        name="head_crossbar",
    )
    frame.visual(
        mesh_from_geometry(_straight_tube([(-0.70, 0.0, 0.18), (-0.88, 0.0, 0.125)], 0.012), "rake_neck"),
        material=rust,
        name="rake_neck",
    )
    frame.visual(
        mesh_from_geometry(_straight_tube([(-0.88, -0.22, 0.125), (-0.88, 0.22, 0.125)], 0.011), "rake_crossbar"),
        material=rust,
        name="rake_crossbar",
    )

    # Curved spring claws behind the wheel. Each claw is rooted in the crossbar
    # and has a small bright worn point where it scrapes soil.
    for i, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        claw_points = [
            (-0.88, y, 0.125),
            (-0.98, y, 0.105),
            (-1.06, y, 0.055),
            (-1.11, y, 0.090),
        ]
        frame.visual(
            mesh_from_geometry(_straight_tube(claw_points, 0.0055, radial_segments=10), f"spring_claw_{i}"),
            material=spring,
            name=f"spring_claw_{i}",
        )
        frame.visual(
            mesh_from_geometry(_straight_tube(claw_points[-2:], 0.0062, radial_segments=10), f"worn_claw_tip_{i}"),
            material=worn,
            name=f"worn_claw_tip_{i}",
        )

    # Visible axle hardware; the wheel hub is captured around this pin.
    frame.visual(
        Cylinder(radius=0.009, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.33), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="axle_pin",
    )
    for y in (-0.116, 0.116):
        frame.visual(
            Cylinder(radius=0.020, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.33), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"axle_nut_{0 if y < 0 else 1}",
        )

    # Raised round bolt heads on the straps, collars, and tine head.
    bolt_locations = [
        (-0.36, -0.116, 0.62),
        (-0.36, 0.116, 0.62),
        (-0.50, -0.116, 0.82),
        (-0.50, 0.116, 0.82),
        (-0.70, -0.085, 0.18),
        (-0.70, 0.085, 0.18),
        (-0.88, -0.225, 0.125),
        (-0.88, 0.225, 0.125),
    ]
    for i, loc in enumerate(bolt_locations):
        frame.visual(Sphere(radius=0.011), origin=Origin(xyz=loc), material=dark, name=f"bolt_head_{i}")

    wheel = model.part("tine_wheel")
    wheel.visual(mesh_from_geometry(_torus_y(0.300, 0.012), "outer_iron_rim"), material=rust, name="outer_iron_rim")
    # Solid iron disc replaces the spoked inner wheel — flat plate filling hub to rim.
    wheel.visual(
        Cylinder(radius=0.290, length=0.008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="iron_disc",
    )
    wheel.visual(
        Cylinder(radius=0.036, length=0.074),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="hub_shell",
    )
    for i in range(10):
        angle = 2.0 * math.pi * i / 10.0 + 0.16
        wheel.visual(mesh_from_geometry(_wheel_lug(angle), f"wheel_lug_{i}"), material=rust, name=f"wheel_lug_{i}")
        if i in (0, 3, 6):
            # small bright caps on several lug tips make the rotating element
            # visibly asymmetric for pose checks and read as worn soil-contact metal.
            c = math.cos(angle)
            s = math.sin(angle)
            tangent = (-s, 0.0, c)
            radial = (c, 0.0, s)
            tip_start = (
                0.334 * radial[0] + 0.018 * tangent[0],
                0.0,
                0.334 * radial[2] + 0.018 * tangent[2],
            )
            tip_end = (
                0.350 * radial[0] + 0.028 * tangent[0],
                0.0,
                0.350 * radial[2] + 0.028 * tangent[2],
            )
            wheel.visual(
                mesh_from_geometry(_straight_tube([tip_start, tip_end], 0.006, radial_segments=10), f"tine_tip_{i}"),
                material=worn,
                name=f"tine_tip_{i}",
            )

    model.articulation(
        "wheel_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def _aabb_center(aabb):
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("tine_wheel")
    axle = object_model.get_articulation("wheel_axle")

    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle_pin",
        elem_b="hub_shell",
        reason="The dark axle pin is intentionally captured inside the rotating wheel hub bore.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a="axle_pin",
        elem_b="iron_disc",
        reason="The solid iron disc surrounds the hub; the axle pin passes through the disc center bore region captured by the hub.",
    )
    ctx.expect_overlap(
        frame,
        wheel,
        axes="y",
        elem_a="axle_pin",
        elem_b="hub_shell",
        min_overlap=0.060,
        name="axle passes through hub",
    )
    ctx.expect_within(
        frame,
        wheel,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="hub_shell",
        margin=0.004,
        name="axle centered in wheel hub",
    )
    ctx.expect_within(
        frame,
        wheel,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="iron_disc",
        margin=0.004,
        name="axle centered in iron disc",
    )

    ctx.check(
        "small class is hand cultivator",
        object_model.meta.get("small_class") == "Hand cultivator" and "cultivator" in object_model.name,
        details=str(object_model.meta),
    )
    ctx.check(
        "has twin handles claws and wheel",
        all(frame.get_visual(name) is not None for name in ("wood_handle_0", "wood_handle_1", "rake_crossbar", "spring_claw_2"))
        and wheel.get_visual("outer_iron_rim") is not None
        and wheel.get_visual("iron_disc") is not None
        and wheel.get_visual("wheel_lug_0") is not None,
    )
    wheel_visual_names = {v.name for v in wheel.visuals}
    ctx.check(
        "wheel is solid disc not spoked",
        "iron_disc" in wheel_visual_names
        and "inner_iron_rim" not in wheel_visual_names
        and not any(f"wheel_spoke_{i}" in wheel_visual_names for i in range(8)),
    )
    ctx.check(
        "wheel is nonfixed rotating mechanism",
        axle.articulation_type == ArticulationType.CONTINUOUS and tuple(round(v, 3) for v in axle.axis) == (0.0, 1.0, 0.0),
        details=f"type={axle.articulation_type}, axis={axle.axis}",
    )

    frame_box = ctx.part_world_aabb(frame)
    wheel_box = ctx.part_world_aabb(wheel)
    ctx.check(
        "cultivator has walk behind proportions",
        frame_box is not None
        and wheel_box is not None
        and frame_box[0][0] < -1.35
        and frame_box[1][2] > 1.18
        and (wheel_box[1][2] - wheel_box[0][2]) > 0.58,
        details=f"frame={frame_box}, wheel={wheel_box}",
    )

    rest_tip = ctx.part_element_world_aabb(wheel, elem="tine_tip_0")
    with ctx.pose({axle: 1.1}):
        moved_tip = ctx.part_element_world_aabb(wheel, elem="tine_tip_0")
    rest_center = _aabb_center(rest_tip) if rest_tip is not None else None
    moved_center = _aabb_center(moved_tip) if moved_tip is not None else None
    ctx.check(
        "wheel lug visibly rotates",
        rest_center is not None
        and moved_center is not None
        and abs(rest_center[0] - moved_center[0]) + abs(rest_center[2] - moved_center[2]) > 0.08,
        details=f"rest={rest_center}, moved={moved_center}",
    )

    return ctx.report()


object_model = build_object_model()
