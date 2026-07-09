from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CapsuleGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quad_cane")

    chrome = Material("brushed_chrome", rgba=(0.78, 0.75, 0.68, 1.0))
    dark_chrome = Material("shadowed_metal", rgba=(0.48, 0.48, 0.46, 1.0))
    black_plastic = Material("black_molded_plastic", rgba=(0.01, 0.01, 0.01, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    # A flared rubber ferrule mesh reused on each of the four cane feet.
    ferrule_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.020, -0.022),
                (0.023, -0.016),
                (0.020, -0.010),
                (0.014, 0.006),
                (0.012, 0.023),
            ],
            segments=40,
            closed=True,
        ),
        "flared_rubber_ferrule",
    )

    # Hollow clamp collar around the telescoping joint, with enough bore
    # clearance to avoid becoming the solid proxy for the sliding tube.
    collar_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.021, -0.026), (0.022, -0.018), (0.022, 0.018), (0.021, 0.026)],
            [(0.012, -0.022), (0.012, 0.022)],
            segments=48,
            start_cap="round",
            end_cap="round",
            lip_samples=5,
        ),
        "height_adjust_collar",
    )
    socket_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.018, -0.016), (0.018, 0.016)],
            [(0.0098, -0.014), (0.0098, 0.014)],
            segments=40,
            start_cap="round",
            end_cap="round",
            lip_samples=4,
        ),
        "lower_shaft_socket",
    )

    base_hub = model.part("base_hub")
    base_hub.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=chrome,
        name="central_plate",
    )
    base_hub.visual(
        socket_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_chrome,
        name="shaft_socket",
    )

    tip_centers = [
        (0.085, 0.083),
        (0.085, -0.083),
        (-0.135, 0.083),
        (-0.135, -0.083),
    ]
    hub_radius = 0.031
    hub_z = 0.080
    ferrule_center_z = -0.058
    ferrule_top_z = ferrule_center_z + 0.023

    for i, (tip_x, tip_y) in enumerate(tip_centers):
        leg = model.part(f"leg_{i}")
        span = math.hypot(tip_x, tip_y)
        start_x = tip_x / span * hub_radius
        start_y = tip_y / span * hub_radius
        mid_x = (start_x + tip_x) * 0.52
        mid_y = (start_y + tip_y) * 0.52
        leg_tube = tube_from_spline_points(
            [
                (start_x, start_y, 0.000),
                (mid_x, mid_y, -0.010),
                (tip_x, tip_y, ferrule_top_z - 0.007),
            ],
            radius=0.006,
            samples_per_segment=18,
            radial_segments=24,
            cap_ends=True,
        )
        leg.visual(
            mesh_from_geometry(leg_tube, f"leg_{i}_tube"),
            material=chrome,
            name="tube",
        )
        leg.visual(
            ferrule_mesh,
            origin=Origin(xyz=(tip_x, tip_y, ferrule_center_z)),
            material=rubber,
            name="rubber_tip",
        )
        model.articulation(
            f"base_to_leg_{i}",
            ArticulationType.FIXED,
            parent=base_hub,
            child=leg,
            origin=Origin(xyz=(0.0, 0.0, hub_z)),
        )

    lower_shaft = model.part("lower_shaft")
    lower_shaft.visual(
        Cylinder(radius=0.009, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=chrome,
        name="lower_tube",
    )
    model.articulation(
        "base_to_lower",
        ArticulationType.FIXED,
        parent=base_hub,
        child=lower_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    collar = model.part("collar")
    collar.visual(collar_mesh, material=chrome, name="clamp_ring")
    collar.visual(
        Cylinder(radius=0.0032, length=0.046),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_chrome,
        name="set_screw",
    )
    collar.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="thumb_knob",
    )
    model.articulation(
        "lower_to_collar",
        ArticulationType.FIXED,
        parent=lower_shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
    )

    upper_shaft = model.part("upper_shaft")
    # The lower 14 cm is intentionally represented as nested inside the lower
    # tube proxy; tests below scope and justify that telescoping overlap.
    upper_shaft.visual(
        Cylinder(radius=0.0062, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=chrome,
        name="upper_tube",
    )
    for hole_i, z in enumerate((-0.080, -0.045, -0.010, 0.025, 0.060)):
        upper_shaft.visual(
            Cylinder(radius=0.0026, length=0.0012),
            origin=Origin(xyz=(0.0062, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name=f"adjust_hole_{hole_i}",
        )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_shaft,
        child=upper_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.10),
    )

    handle = model.part("handle")
    grip_mesh = CapsuleGeometry(radius=0.020, length=0.120, radial_segments=32).rotate_y(
        math.pi / 2.0
    )
    grip_mesh.translate(0.0, 0.0, 0.047)
    handle.visual(
        mesh_from_geometry(grip_mesh, "ergonomic_t_grip"),
        material=black_plastic,
        name="t_grip",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_plastic,
        name="neck",
    )
    handle.visual(
        Sphere(radius=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=black_plastic,
        name="palm_swell",
    )
    model.articulation(
        "upper_to_handle",
        ArticulationType.FIXED,
        parent=upper_shaft,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shaft")
    upper = object_model.get_part("upper_shaft")
    handle = object_model.get_part("handle")
    collar = object_model.get_part("collar")
    base = object_model.get_part("base_hub")
    slide = object_model.get_articulation("lower_to_upper")

    ctx.allow_overlap(
        lower,
        upper,
        elem_a="lower_tube",
        elem_b="upper_tube",
        reason=(
            "The upper chrome tube is intentionally modeled as a nested sliding "
            "member inside the simplified solid lower telescoping sleeve."
        ),
    )

    leg_parts = [object_model.get_part(f"leg_{i}") for i in range(4)]
    ctx.check("four named base legs", len(leg_parts) == 4)
    for i, leg in enumerate(leg_parts):
        ctx.allow_overlap(
            base,
            leg,
            elem_a="central_plate",
            elem_b="tube",
            reason="Each chrome leg tube is intentionally socketed into the central base casting.",
        )
        ctx.expect_overlap(
            base,
            leg,
            axes="z",
            elem_a="central_plate",
            elem_b="tube",
            min_overlap=0.004,
            name=f"leg_{i} tube is captured by base casting",
        )
    ctx.allow_overlap(
        collar,
        lower,
        elem_a="set_screw",
        elem_b="lower_tube",
        reason="The height-adjust set screw is intentionally seated into the lower tube wall.",
    )
    ctx.expect_gap(
        collar,
        lower,
        axis="x",
        positive_elem="set_screw",
        negative_elem="lower_tube",
        max_gap=0.001,
        max_penetration=0.002,
        name="set screw bears on lower tube",
    )

    ctx.expect_contact(
        base,
        lower,
        elem_a="central_plate",
        elem_b="lower_tube",
        contact_tol=0.002,
        name="lower shaft seats on base hub",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="lower_tube",
        margin=0.001,
        name="telescoping tube centered in lower sleeve",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_tube",
        min_overlap=0.12,
        name="collapsed tube remains deeply inserted",
    )
    ctx.expect_within(
        upper,
        collar,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="clamp_ring",
        margin=0.0,
        name="collar surrounds sliding tube",
    )

    def part_bounds(parts):
        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]
        for part in parts:
            bounds = ctx.part_world_aabb(part)
            if bounds is None:
                return None
            lo, hi = bounds
            for axis in range(3):
                mins[axis] = min(mins[axis], lo[axis])
                maxs[axis] = max(maxs[axis], hi[axis])
        return mins, maxs

    footprint = part_bounds(leg_parts)
    if footprint is None:
        ctx.fail("wide quad base footprint", "could not compute leg bounds")
    else:
        lo, hi = footprint
        x_span = hi[0] - lo[0]
        y_span = hi[1] - lo[1]
        ctx.check(
            "wide quad base footprint",
            0.24 <= x_span <= 0.30 and 0.18 <= y_span <= 0.23,
            details=f"span=({x_span:.3f}, {y_span:.3f})",
        )
        ctx.check(
            "base offset to one side of shaft",
            abs(lo[0]) > hi[0] + 0.035,
            details=f"x bounds=({lo[0]:.3f}, {hi[0]:.3f})",
        )
        ctx.check(
            "rubber tips rest on ground",
            abs(lo[2]) <= 0.003,
            details=f"lowest tip z={lo[2]:.4f}",
        )

    handle_bounds = ctx.part_world_aabb(handle)
    ctx.check(
        "resting cane height is about 90 cm",
        handle_bounds is not None and 0.88 <= handle_bounds[1][2] <= 0.94,
        details=f"handle_bounds={handle_bounds}",
    )

    rest_pos = ctx.part_world_position(handle)
    with ctx.pose({slide: 0.10}):
        ctx.expect_within(
            upper,
            lower,
            axes="xy",
            inner_elem="upper_tube",
            outer_elem="lower_tube",
            margin=0.001,
            name="extended tube remains centered",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="upper_tube",
            elem_b="lower_tube",
            min_overlap=0.035,
            name="extended tube remains inserted",
        )
        extended_pos = ctx.part_world_position(handle)

    ctx.check(
        "prismatic height adjustment raises handle",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.09,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
