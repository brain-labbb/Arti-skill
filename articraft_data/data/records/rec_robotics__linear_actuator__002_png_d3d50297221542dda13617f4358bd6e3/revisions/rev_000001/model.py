from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cut_x_holes(body: cq.Workplane, thickness: float, holes: list[tuple[float, float, float]]) -> cq.Workplane:
    """Cut circular through holes whose axes run along global X."""
    for y, z, radius in holes:
        cutter = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(radius)
            .extrude(thickness * 4.0, both=True)
        )
        body = body.cut(cutter)
    return body


def _plate_with_holes(
    *,
    thickness: float,
    width: float,
    height: float,
    holes: list[tuple[float, float, float]],
) -> cq.Workplane:
    plate = cq.Workplane("XY").box(thickness, width, height)
    plate = _cut_x_holes(plate, thickness, holes)
    return plate


def _motor_block() -> cq.Workplane:
    holes = [
        (0.0, 0.020, 0.018),
        (-0.055, -0.040, 0.006),
        (0.055, -0.040, 0.006),
        (-0.055, 0.052, 0.006),
        (0.055, 0.052, 0.006),
        (-0.066, 0.000, 0.004),
        (0.066, 0.000, 0.004),
    ]
    block = cq.Workplane("XY").box(0.100, 0.155, 0.135)
    return _cut_x_holes(block, 0.100, holes)


def _base_rail() -> cq.Workplane:
    """Long aluminum rail with shallow T-slot style grooves."""
    length, width, height = 0.830, 0.115, 0.050
    rail = cq.Workplane("XY").box(length, width, height)
    # Three top grooves and two side reliefs make the rail read like an
    # extruded linear-actuator profile rather than a plain block.
    for y in (-0.036, 0.0, 0.036):
        groove = cq.Workplane("XY").box(length + 0.010, 0.012, 0.026).translate((0.0, y, height / 2.0))
        rail = rail.cut(groove)
    for y in (-width / 2.0, width / 2.0):
        slot = cq.Workplane("XY").box(length + 0.010, 0.010, 0.018).translate((0.0, y, 0.002))
        rail = rail.cut(slot)
    return rail


def _carriage_block() -> cq.Workplane:
    """Sliding carriage with real through-holes for rods, screw, and mounts."""
    block = cq.Workplane("XY").box(0.120, 0.142, 0.095)
    holes = [
        (-0.045, 0.000, 0.0115),  # guide rod clearance
        (0.045, 0.000, 0.0115),   # guide rod clearance
        (0.000, 0.000, 0.0135),   # lead screw clearance
        (-0.060, -0.028, 0.0048),
        (0.060, -0.028, 0.0048),
        (-0.060, 0.030, 0.0048),
        (0.060, 0.030, 0.0048),
    ]
    block = _cut_x_holes(block, 0.120, holes)
    # Underside slide relief clears the raised center of the rail.
    channel = cq.Workplane("XY").box(0.130, 0.068, 0.030).translate((0.0, 0.0, -0.047))
    block = block.cut(channel)
    return block


def _tube_along_x(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length, both=True)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length * 1.25, both=True)
    return outer.cut(inner)


def _tube_geometry_z(outer_radius: float, inner_radius: float, length: float) -> LatheGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=48,
    )


def _lead_screw_thread_mesh(length: float = 0.760):
    turns = 26
    samples = turns * 12
    radius = 0.0066
    points = []
    for i in range(samples + 1):
        t = i / samples
        x = -length / 2.0 + length * t
        theta = 2.0 * pi * turns * t
        points.append((x, radius * cos(theta), radius * sin(theta)))
    return tube_from_spline_points(
        points,
        radius=0.0011,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="linear_actuator_rail_stage",
        meta={
            "category": "Robotics / Linear actuator",
            "run_note": "Reference matches a linear actuator/rail stage; no classification mismatch suspected.",
        },
    )

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    bright_aluminum = model.material("polished_aluminum", rgba=(0.90, 0.92, 0.88, 1.0))
    black = model.material("black_anodized", rgba=(0.015, 0.016, 0.016, 1.0))
    dark = model.material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("machined_steel", rgba=(0.82, 0.83, 0.80, 1.0))
    belt = model.material("black_timing_belt", rgba=(0.045, 0.045, 0.040, 1.0))
    brass = model.material("bronze_lead_nut", rgba=(0.78, 0.54, 0.24, 1.0))

    frame = model.part("frame")

    frame.visual(
        mesh_from_cadquery(_base_rail(), "extruded_base_rail", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=aluminum,
        name="base_rail",
    )

    end_holes = [
        (-0.045, 0.020, 0.0080),
        (0.045, 0.020, 0.0080),
        (0.000, 0.020, 0.0140),
        (-0.064, -0.043, 0.0060),
        (0.064, -0.043, 0.0060),
        (-0.064, 0.064, 0.0060),
        (0.064, 0.064, 0.0060),
    ]
    for index, x in enumerate((-0.420, 0.420)):
        frame.visual(
            mesh_from_cadquery(
                _plate_with_holes(thickness=0.025, width=0.170, height=0.160, holes=end_holes),
                f"end_plate_{index}_mesh",
                tolerance=0.0008,
            ),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=black,
            name=f"end_plate_{index}",
        )

    frame.visual(
        mesh_from_cadquery(_motor_block(), "motor_block_mesh", tolerance=0.0008),
        origin=Origin(xyz=(-0.4825, 0.0, 0.080)),
        material=black,
        name="motor_block",
    )

    # Guide rods are fixed to the frame and run through the carriage bushings.
    for index, y in enumerate((-0.045, 0.045)):
        frame.visual(
            Cylinder(radius=0.0070, length=0.850),
            origin=Origin(xyz=(0.0, y, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
            material=bright_aluminum,
            name=f"guide_rod_{index}",
        )
        for x in (-0.420, 0.420):
            frame.visual(
                Cylinder(radius=0.0120, length=0.012),
                origin=Origin(xyz=(x, y, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
                material=steel,
                name=f"rod_clamp_{index}_{0 if x < 0.0 else 1}",
            )

    # Static bearing rings around the screw holes on both end plates.
    for index, x in enumerate((-0.420, 0.420)):
        frame.visual(
            mesh_from_geometry(_tube_geometry_z(0.023, 0.011, 0.010), f"bearing_ring_{index}_mesh"),
            origin=Origin(xyz=(x, 0.0, 0.100), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"bearing_ring_{index}",
        )

    # A front timing-belt/rack strip echoes the toothed drive strip visible in
    # the reference while remaining attached to the front side of the rail.
    frame.visual(
        Box((0.730, 0.012, 0.006)),
        origin=Origin(xyz=(0.030, -0.0635, 0.012)),
        material=belt,
        name="belt_backing",
    )
    tooth_count = 58
    for i in range(tooth_count):
        x = -0.330 + i * (0.660 / (tooth_count - 1))
        frame.visual(
            Box((0.0065, 0.010, 0.0045)),
            origin=Origin(xyz=(x, -0.0635, 0.01725)),
            material=belt,
            name=f"belt_tooth_{i}",
        )

    # Top mounting holes on the aluminum rail, shown as black recessed circles.
    for index, x in enumerate((-0.330, -0.210, -0.090, 0.090, 0.210, 0.330)):
        frame.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.051)),
            material=dark,
            name=f"rail_hole_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_block(), "carriage_block_mesh", tolerance=0.0008),
        material=black,
        name="carriage_block",
    )
    carriage.visual(
        mesh_from_geometry(_tube_geometry_z(0.021, 0.0115, 0.010), "lead_nut_ring_mesh"),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lead_nut_ring",
    )
    # Bright caps on top of the carriage read as socket-head fasteners.
    for index, (x, y) in enumerate(((-0.042, -0.052), (0.042, -0.052), (-0.042, 0.052), (0.042, 0.052))):
        carriage.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, y, 0.0495)),
            material=steel,
            name=f"carriage_bolt_{index}",
        )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        Cylinder(radius=0.0060, length=0.840),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="screw_core",
    )
    lead_screw.visual(
        mesh_from_geometry(_lead_screw_thread_mesh(), "lead_screw_thread"),
        material=bright_aluminum,
        name="screw_thread",
    )
    lead_screw.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.468, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="drive_coupler",
    )
    lead_screw.visual(
        Cylinder(radius=0.0050, length=0.105),
        origin=Origin(xyz=(-0.420, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="shaft_extension",
    )
    for index, x in enumerate((-0.420, 0.420)):
        lead_screw.visual(
            Cylinder(radius=0.0110, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"bearing_journal_{index}",
        )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(-0.160, 0.0, 0.0975)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.420),
    )

    model.articulation(
        "screw_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-6.283185307, upper=6.283185307),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    screw = object_model.get_part("lead_screw")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("screw_spin")

    ctx.check(
        "linear actuator part set",
        all(object_model.get_part(name) is not None for name in ("frame", "carriage", "lead_screw")),
        details="Expected frame, sliding carriage, and exposed lead screw parts.",
    )
    ctx.check(
        "primary joints present",
        slide.articulation_type == ArticulationType.PRISMATIC and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, spin={spin.articulation_type}",
    )
    ctx.check(
        "joint axes follow rail length",
        tuple(slide.axis) == (1.0, 0.0, 0.0) and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"slide_axis={slide.axis}, spin_axis={spin.axis}",
    )

    for index in (0, 1):
        ctx.allow_overlap(
            frame,
            screw,
            elem_a=f"bearing_ring_{index}",
            elem_b=f"bearing_journal_{index}",
            reason="The rotating screw journal is intentionally seated in the end-plate bearing ring.",
        )
        ctx.expect_overlap(
            frame,
            screw,
            axes="x",
            elem_a=f"bearing_ring_{index}",
            elem_b=f"bearing_journal_{index}",
            min_overlap=0.006,
            name=f"bearing journal {index} is retained axially",
        )
        ctx.expect_within(
            screw,
            frame,
            axes="yz",
            inner_elem=f"bearing_journal_{index}",
            outer_elem=f"bearing_ring_{index}",
            margin=0.001,
            name=f"bearing journal {index} is centered in bearing ring",
        )

    ctx.expect_within(
        carriage,
        frame,
        axes="x",
        inner_elem="carriage_block",
        outer_elem="base_rail",
        margin=0.010,
        name="carriage stays on rail at rest",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="base_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage rides on base rail",
    )
    ctx.expect_within(
        screw,
        carriage,
        axes="yz",
        inner_elem="screw_core",
        outer_elem="carriage_block",
        margin=0.0,
        name="lead screw passes through carriage bore",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.420, spin: pi}):
        ctx.expect_within(
            carriage,
            frame,
            axes="x",
            inner_elem="carriage_block",
            outer_elem="base_rail",
            margin=0.010,
            name="carriage remains on rail at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive slide extends carriage along rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.35,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
