from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    z_min: float,
) -> cq.Workplane:
    """CadQuery annular disk in model meters, extruded along +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z_min))
    )


def _through_cylinder(
    radius: float,
    z_min: float,
    height: float,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _housing_shell() -> cq.Workplane:
    """Stepped outer housing with a real central bore and radial bolt pattern."""
    body = _annular_cylinder(0.076, 0.032, 0.018, -0.018)
    body = body.union(_annular_cylinder(0.064, 0.0335, 0.042, -0.060))
    body = body.union(_annular_cylinder(0.054, 0.035, 0.0055, 0.000))
    body = body.union(_annular_cylinder(0.0348, 0.032, 0.0050, 0.000))

    # Through-bolts around the front flange, matching the photographed pattern.
    for i in range(14):
        a = 2.0 * math.pi * i / 14.0 + math.radians(6.0)
        x = 0.063 * math.cos(a)
        y = 0.063 * math.sin(a)
        body = body.cut(_through_cylinder(0.0038, -0.022, 0.030, x, y))

    return body


def _output_flange() -> cq.Workplane:
    """Rotating output disk with central aperture, keyway notch and bolt holes."""
    flange = _annular_cylinder(0.0285, 0.0105, 0.010, 0.010)
    # Rear bearing land: this is the rotating inner race that touches the
    # captured ball set carried by the fixed housing.
    flange = flange.union(_annular_cylinder(0.03950, 0.0200, 0.0038, 0.0065))

    for i in range(4):
        a = 2.0 * math.pi * i / 4.0 + math.radians(22.5)
        x = 0.019 * math.cos(a)
        y = 0.019 * math.sin(a)
        flange = flange.cut(_through_cylinder(0.00215, 0.007, 0.017, x, y))

    # A small broached key relief in the input bore gives the flange a realistic,
    # asymmetrical machined detail without adding text or decorative props.
    key_slot = (
        cq.Workplane("XY")
        .box(0.0048, 0.0070, 0.016, centered=True)
        .translate((0.0, 0.0105, 0.015))
    )
    flange = flange.cut(key_slot)
    return flange


def _input_wave_generator() -> cq.Workplane:
    """Elliptical wave-generator cam visible through the central aperture."""
    return (
        cq.Workplane("XY")
        .ellipse(0.0150, 0.0064)
        .extrude(0.0035)
        .translate((0.0, 0.0, 0.0205))
    )


def _thin_ring(outer_radius: float, inner_radius: float, z_min: float, thickness: float) -> cq.Workplane:
    return _annular_cylinder(outer_radius, inner_radius, thickness, z_min)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="harmonic_drive_core",
        meta={
            "category": "Robotics / Harmonic drive",
            "notes": "Reference image and category both read as a harmonic/strain-wave gearbox.",
        },
    )

    aluminum = model.material("brushed_aluminum", rgba=(0.82, 0.82, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    polished = model.material("polished_bearing_steel", rgba=(0.92, 0.92, 0.88, 1.0))
    shadow = model.material("machined_shadow", rgba=(0.015, 0.016, 0.018, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell", tolerance=0.00045, angular_tolerance=0.05),
        material=aluminum,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_thin_ring(0.0565, 0.0535, -0.0001, 0.0010), "outer_groove", tolerance=0.00035),
        material=shadow,
        name="outer_groove",
    )
    housing.visual(
        mesh_from_cadquery(_thin_ring(0.0470, 0.0342, 0.0034, 0.0018), "bearing_cage", tolerance=0.00035),
        material=dark_steel,
        name="bearing_cage",
    )
    housing.visual(
        mesh_from_cadquery(_thin_ring(0.0473, 0.0433, 0.0015, 0.0038), "bearing_outer_race", tolerance=0.00035),
        material=dark_steel,
        name="bearing_outer_race",
    )
    housing.visual(
        mesh_from_cadquery(_thin_ring(0.0394, 0.0367, 0.0015, 0.0038), "bearing_inner_race", tolerance=0.00035),
        material=dark_steel,
        name="bearing_inner_race",
    )
    housing.visual(
        mesh_from_cadquery(_thin_ring(0.0373, 0.0344, 0.0010, 0.0022), "circular_spline_line", tolerance=0.00035),
        material=shadow,
        name="circular_spline_line",
    )

    # Captured balls in the visible front bearing.  They are authored on the
    # fixed housing race so they read as a retained bearing set rather than as
    # free floating decorative beads.
    ball_count = 28
    for i in range(ball_count):
        a = 2.0 * math.pi * i / ball_count
        housing.visual(
            Sphere(0.00255),
            origin=Origin(xyz=(0.0420 * math.cos(a), 0.0420 * math.sin(a), 0.0060)),
            material=polished,
            name=f"bearing_ball_{i:02d}",
        )

    # Small dark teeth just inside the bearing suggest the circular spline /
    # flexspline interface visible in many harmonic drive front faces.
    tooth_count = 42
    for i in range(tooth_count):
        a = 2.0 * math.pi * i / tooth_count
        r = 0.0357
        tooth = Cylinder(radius=0.0009, length=0.0030)
        housing.visual(
            tooth,
            origin=Origin(xyz=(r * math.cos(a), r * math.sin(a), 0.0030)),
            material=dark_steel,
            name=f"spline_tooth_{i:02d}",
        )

    output = model.part("output_flange")
    output.visual(
        mesh_from_cadquery(_output_flange(), "output_flange", tolerance=0.00035, angular_tolerance=0.05),
        material=aluminum,
        name="output_flange",
    )
    output.visual(
        mesh_from_cadquery(_thin_ring(0.0260, 0.0242, 0.0200, 0.0008), "output_face_groove", tolerance=0.0003),
        material=shadow,
        name="output_face_groove",
    )
    for i in range(4):
        a = 2.0 * math.pi * i / 4.0 + math.radians(22.5)
        output.visual(
            Cylinder(radius=0.0023, length=0.0012),
            origin=Origin(xyz=(0.019 * math.cos(a), 0.019 * math.sin(a), 0.0104)),
            material=shadow,
            name=f"output_hole_shadow_{i}",
        )

    input_hub = model.part("input_hub")
    input_hub.visual(
        mesh_from_cadquery(_thin_ring(0.0128, 0.0046, 0.0200, 0.0022), "input_retaining_collar", tolerance=0.00025),
        material=shadow,
        name="input_retaining_collar",
    )
    input_hub.visual(
        mesh_from_cadquery(_thin_ring(0.0074, 0.0046, 0.0110, 0.0140), "input_sleeve", tolerance=0.00025),
        material=shadow,
        name="input_sleeve",
    )

    # Wave generator cam seated directly in the input hub (pure rotary reducer,
    # no vertical lift mechanism).
    input_hub.visual(
        mesh_from_cadquery(_input_wave_generator(), "wave_generator", tolerance=0.00025, angular_tolerance=0.04),
        material=dark_steel,
        name="wave_generator",
    )

    model.articulation(
        "housing_to_output",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=output,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
        meta={"role": "coaxial output flange continuous rotation at reduced speed"},
    )
    model.articulation(
        "housing_to_input",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=input_hub,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=60.0),
        meta={"role": "input wave-generator continuous high-speed rotation"},
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    output = object_model.get_part("output_flange")
    input_hub = object_model.get_part("input_hub")
    output_joint = object_model.get_articulation("housing_to_output")
    input_joint = object_model.get_articulation("housing_to_input")

    ctx.check(
        "pure rotary harmonic drive assembly",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=f"parts={len(object_model.parts)} joints={len(object_model.articulations)}",
    )
    ctx.check(
        "housing_to_output and housing_to_input are CONTINUOUS coaxial joints",
        tuple(output_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(input_joint.axis) == (0.0, 0.0, 1.0)
        and output_joint.articulation_type == ArticulationType.CONTINUOUS
        and input_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"output={output_joint.axis}/{output_joint.articulation_type}, "
            f"input={input_joint.axis}/{input_joint.articulation_type}"
        ),
    )
    ctx.check(
        "continuous joints have no positional bounds",
        output_joint.motion_limits.lower is None
        and output_joint.motion_limits.upper is None
        and input_joint.motion_limits.lower is None
        and input_joint.motion_limits.upper is None,
        details=(
            f"output bounds=[{output_joint.motion_limits.lower}, {output_joint.motion_limits.upper}], "
            f"input bounds=[{input_joint.motion_limits.lower}, {input_joint.motion_limits.upper}]"
        ),
    )
    ctx.check(
        "visible bolt and bearing pattern",
        len([v for v in housing.visuals if v.name and v.name.startswith("bearing_ball_")]) == 28
        and len([v for v in housing.visuals if v.name and v.name.startswith("spline_tooth_")]) == 42
        and len([v for v in output.visuals if v.name and v.name.startswith("output_hole_shadow_")]) == 4,
        details="expected 28 bearing balls, 42 spline hints, and 4 output bolt holes",
    )
    ctx.expect_within(
        input_hub,
        output,
        axes="xy",
        margin=0.0,
        name="input hub is nested inside output flange footprint",
    )
    ctx.expect_contact(
        output,
        housing,
        contact_tol=0.001,
        name="output inner race is supported by bearing balls",
    )
    ctx.expect_contact(
        input_hub,
        output,
        contact_tol=0.001,
        name="input collar is seated on the output bore",
    )

    # Prove the continuous input rotates the elliptical wave generator:
    # at rest the cam major axis is along X; after pi/2 rotation the AABB
    # X-extent must change noticeably.
    rest_aabb = ctx.part_world_aabb(input_hub)
    with ctx.pose({input_joint: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(input_hub)

    if rest_aabb is not None and turned_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        turned_dx = turned_aabb[1][0] - turned_aabb[0][0]
        ctx.check(
            "continuous input joint rotates elliptical wave generator",
            abs(rest_dx - turned_dx) > 0.002,
            details=f"rest_dx={rest_dx:.5f}, turned_dx={turned_dx:.5f}",
        )
    else:
        ctx.fail("continuous input joint rotates elliptical wave generator", "input hub AABB unavailable")

    # Prove the continuous output flange actually rotates independently by
    # tracking a specific bolt-hole shadow element whose position is off-axis.
    rest_bolt = ctx.part_element_world_aabb(output, elem="output_hole_shadow_0")
    with ctx.pose({output_joint: math.pi}):
        turned_bolt = ctx.part_element_world_aabb(output, elem="output_hole_shadow_0")

    if rest_bolt is not None and turned_bolt is not None:
        rest_cx = 0.5 * (rest_bolt[0][0] + rest_bolt[1][0])
        turned_cx = 0.5 * (turned_bolt[0][0] + turned_bolt[1][0])
        ctx.check(
            "continuous output joint rotates flange bolt pattern",
            abs(rest_cx - turned_cx) > 0.005,
            details=f"rest_cx={rest_cx:.5f}, turned_cx={turned_cx:.5f}",
        )
    else:
        ctx.fail("continuous output joint rotates flange bolt pattern", "bolt hole shadow AABB unavailable")

    return ctx.report()


object_model = build_object_model()
