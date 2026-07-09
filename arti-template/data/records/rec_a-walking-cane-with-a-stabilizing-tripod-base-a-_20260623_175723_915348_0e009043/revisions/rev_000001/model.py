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


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Open annular tube centered on local Z."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0005, angular_tolerance=0.08)


def _cylinder_between(start, end):
    """Return (length, origin) for a SDK cylinder whose local Z spans start->end."""
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    yaw = math.atan2(vy, vx)
    pitch = math.acos(vz / length)
    midpoint = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    return length, Origin(xyz=midpoint, rpy=(0.0, pitch, yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_walking_cane")

    chrome = Material("brushed_aluminium", rgba=(0.78, 0.80, 0.78, 1.0))
    bright_chrome = Material("polished_highlight", rgba=(0.95, 0.96, 0.93, 1.0))
    dark_groove = Material("dark_recess", rgba=(0.015, 0.014, 0.013, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    collar_metal = Material("collar_metal", rgba=(0.70, 0.72, 0.70, 1.0))

    # Root lower assembly: lower telescoping sleeve, collar, and the tripod hub.
    lower_sleeve = model.part("lower_sleeve")
    lower_sleeve.visual(
        _tube_mesh(0.0095, 0.0075, 0.420, "lower_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=chrome,
        name="lower_tube",
    )
    lower_sleeve.visual(
        _tube_mesh(0.0150, 0.0078, 0.045, "height_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=collar_metal,
        name="height_collar",
    )
    lower_sleeve.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=collar_metal,
        name="tripod_hub",
    )
    lower_sleeve.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=collar_metal,
        name="rounded_hub",
    )
    lower_sleeve.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.028, 0.0, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_metal,
        name="clamp_screw",
    )
    lower_sleeve.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.045, 0.0, 0.500), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="collar_knob",
    )
    lower_sleeve.visual(
        Box((0.002, 0.001, 0.390)),
        origin=Origin(xyz=(0.000, 0.0098, 0.335)),
        material=bright_chrome,
        name="lower_reflection",
    )

    # Sliding upper telescoping member.  Its frame is at the collar entry plane.
    upper_shaft = model.part("upper_shaft")
    upper_shaft.visual(
        Cylinder(radius=0.0065, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="upper_tube",
    )
    upper_shaft.visual(
        Box((0.0015, 0.0009, 0.400)),
        origin=Origin(xyz=(0.0, 0.0068, 0.115)),
        material=bright_chrome,
        name="upper_reflection",
    )

    # Push-button adjustment holes on the visible upper tube.
    for hole_index, z in enumerate((0.045, 0.080, 0.115, 0.150, 0.185)):
        upper_shaft.visual(
            Cylinder(radius=0.0032, length=0.0016),
            origin=Origin(xyz=(0.0, 0.0071, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_groove,
            name=f"button_hole_{hole_index}",
        )
    upper_shaft.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(0.0, 0.0095, 0.028)),
        material=bright_chrome,
        name="spring_button",
    )

    model.articulation(
        "collar_slide",
        ArticulationType.PRISMATIC,
        parent=lower_sleeve,
        child=upper_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.15, lower=0.0, upper=0.080),
    )

    # Separate black T-handle fixed to the sliding upper shaft.
    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.022, length=0.178),
        origin=Origin(xyz=(0.0, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="t_grip",
    )
    handle.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(-0.089, 0.0, 0.058)),
        material=black_rubber,
        name="grip_end_0",
    )
    handle.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.089, 0.0, 0.058)),
        material=black_rubber,
        name="grip_end_1",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_rubber,
        name="handle_socket",
    )
    for band_index, x in enumerate((-0.052, -0.026, 0.026, 0.052)):
        handle.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=Material("subtle_grip_ridge", rgba=(0.018, 0.018, 0.016, 1.0)),
            name=f"grip_ridge_{band_index}",
        )
    model.articulation(
        "shaft_to_handle",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.338)),
    )

    # Three splayed stabilizing legs emitted with a loop.  Each leg part carries
    # its chrome tube and black rubber ferrule so the base reads as a true tripod.
    foot_radius = 0.115
    hub_radius = 0.026
    for leg_index in range(3):
        angle = math.pi / 2.0 + leg_index * 2.0 * math.pi / 3.0
        radial = (math.cos(angle), math.sin(angle))
        start_world = (hub_radius * radial[0], hub_radius * radial[1], 0.112)
        foot_world = (foot_radius * radial[0], foot_radius * radial[1], 0.0175)
        end_local = (
            foot_world[0] - start_world[0],
            foot_world[1] - start_world[1],
            0.032 - start_world[2],
        )

        leg = model.part(f"leg_{leg_index}")
        tube_len, tube_origin = _cylinder_between((0.0, 0.0, 0.0), end_local)
        leg.visual(
            Cylinder(radius=0.0058, length=tube_len),
            origin=tube_origin,
            material=chrome,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.017, length=0.032),
            origin=Origin(xyz=(foot_world[0] - start_world[0], foot_world[1] - start_world[1], 0.016 - start_world[2])),
            material=black_rubber,
            name="rubber_ferrule",
        )
        leg.visual(
            Cylinder(radius=0.021, length=0.008),
            origin=Origin(xyz=(foot_world[0] - start_world[0], foot_world[1] - start_world[1], 0.004 - start_world[2])),
            material=black_rubber,
            name="flared_foot",
        )
        leg.visual(
            Box((0.0016, 0.0008, tube_len * 0.72)),
            origin=Origin(
                xyz=(end_local[0] * 0.42, end_local[1] * 0.42, end_local[2] * 0.42),
                rpy=tube_origin.rpy,
            ),
            material=bright_chrome,
            name="leg_reflection",
        )
        model.articulation(
            f"base_to_leg_{leg_index}",
            ArticulationType.FIXED,
            parent=lower_sleeve,
            child=leg,
            origin=Origin(xyz=start_world),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_sleeve")
    upper = object_model.get_part("upper_shaft")
    handle = object_model.get_part("handle")
    slide = object_model.get_articulation("collar_slide")

    ctx.allow_overlap(
        handle,
        upper,
        elem_a="handle_socket",
        elem_b="upper_tube",
        reason="The molded T-handle socket intentionally cups over the top of the chrome tube.",
    )
    ctx.expect_overlap(
        handle,
        upper,
        axes="z",
        elem_a="handle_socket",
        elem_b="upper_tube",
        min_overlap=0.006,
        name="handle socket captures the tube end",
    )

    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="height_collar",
        margin=0.001,
        name="upper tube is centered through the collar",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_tube",
        min_overlap=0.080,
        name="collapsed upper tube remains inserted in the lower sleeve",
    )

    rest_handle = ctx.part_world_position(handle)
    rest_upper = ctx.part_world_position(upper)
    with ctx.pose({slide: 0.080}):
        extended_handle = ctx.part_world_position(handle)
        extended_upper = ctx.part_world_position(upper)
        ctx.expect_within(
            upper,
            lower,
            axes="xy",
            inner_elem="upper_tube",
            outer_elem="height_collar",
            margin=0.001,
            name="extended upper tube remains centered through the collar",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="upper_tube",
            elem_b="lower_tube",
            min_overlap=0.060,
            name="extended upper tube keeps retained insertion",
        )
    ctx.check(
        "collar slide raises handle",
        rest_handle is not None
        and extended_handle is not None
        and rest_upper is not None
        and extended_upper is not None
        and extended_handle[2] > rest_handle[2] + 0.075
        and extended_upper[2] > rest_upper[2] + 0.075,
        details=f"rest_handle={rest_handle}, extended_handle={extended_handle}",
    )

    leg_names = [f"leg_{i}" for i in range(3)]
    ctx.check(
        "three tripod legs are named",
        all(object_model.get_part(name) is not None for name in leg_names),
        details=str(leg_names),
    )
    leg_positions = []
    for name in leg_names:
        leg = object_model.get_part(name)
        ctx.allow_overlap(
            leg,
            lower,
            elem_a="leg_tube",
            elem_b="rounded_hub",
            reason="Each splayed chrome leg is intentionally inserted into the rounded tripod hub.",
        )
        ctx.allow_overlap(
            leg,
            lower,
            elem_a="leg_tube",
            elem_b="tripod_hub",
            reason="Each splayed chrome leg is intentionally seated in the tripod hub socket.",
        )
        ctx.expect_overlap(
            leg,
            lower,
            axes="z",
            elem_a="leg_tube",
            elem_b="tripod_hub",
            min_overlap=0.001,
            name=f"{name} enters the tripod hub height",
        )
        aabb = ctx.part_element_world_aabb(leg, elem="rubber_ferrule")
        if aabb is not None:
            leg_positions.append(((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5))

    if len(leg_positions) == 3:
        max_span = 0.0
        for i in range(3):
            for j in range(i + 1, 3):
                dx = leg_positions[i][0] - leg_positions[j][0]
                dy = leg_positions[i][1] - leg_positions[j][1]
                max_span = max(max_span, math.sqrt(dx * dx + dy * dy))
        ctx.check(
            "tripod base is about twenty centimeters across",
            0.16 <= max_span <= 0.24,
            details=f"measured_span={max_span:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
