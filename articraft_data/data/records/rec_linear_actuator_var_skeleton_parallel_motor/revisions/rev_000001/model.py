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
    """Compact NEMA-23-style stepper motor body with flange and shaft stub."""
    body = cq.Workplane("XY").box(0.082, 0.070, 0.070)
    # Front mounting flange (slightly wider than body)
    flange = cq.Workplane("XY").box(0.008, 0.082, 0.082).translate((0.045, 0.0, 0.0))
    body = body.union(flange)
    # Shaft stub extending in +X from the flange face
    shaft = (
        cq.Workplane("YZ")
        .circle(0.004)
        .extrude(0.020)
        .translate((0.049, 0.0, 0.0))
    )
    body = body.union(shaft)
    # Rear connector cap
    cap = cq.Workplane("XY").box(0.012, 0.050, 0.050).translate((-0.047, 0.0, 0.0))
    body = body.union(cap)
    # Mounting holes on flange
    for y, z in ((-0.032, -0.032), (0.032, -0.032), (-0.032, 0.032), (0.032, 0.032)):
        hole = (
            cq.Workplane("YZ")
            .circle(0.003)
            .extrude(0.012, both=True)
            .translate((0.045, y, z))
        )
        body = body.cut(hole)
    return body


def _motor_bracket() -> cq.Workplane:
    """Flat motor mounting plate with lightening holes."""
    plate = cq.Workplane("XY").box(0.008, 0.160, 0.130)
    # Large lightening hole
    hole = (
        cq.Workplane("YZ")
        .center(-0.080, 0.030)
        .circle(0.018)
        .extrude(0.012, both=True)
    )
    plate = plate.cut(hole)
    # Screw pass-through hole (center of plate) — sized for the screw pulley
    screw_hole = (
        cq.Workplane("YZ")
        .center(0.050, 0.040)
        .circle(0.028)
        .extrude(0.012, both=True)
    )
    plate = plate.cut(screw_hole)
    # Motor mounting holes (NEMA-23 pattern)
    for y, z in ((-0.082, 0.008), (-0.082, 0.072), (-0.118, 0.008), (-0.118, 0.072)):
        h = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(0.003)
            .extrude(0.012, both=True)
        )
        plate = plate.cut(h)
    return plate


def _timing_pulley(outer_r: float, bore_r: float, width: float) -> cq.Workplane:
    """Timing pulley with flanges and center bore."""
    body = cq.Workplane("YZ").circle(outer_r).extrude(width / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(bore_r).extrude(width, both=True)
    body = body.cut(bore)
    # Flanges at both ends
    flange_r = outer_r + 0.003
    flange_half = 0.001
    for dx in (-width / 2.0, width / 2.0):
        flange = (
            cq.Workplane("YZ")
            .circle(flange_r)
            .extrude(flange_half, both=True)
            .translate((dx, 0.0, 0.0))
        )
        body = body.union(flange)
    return body


def _reduction_belt_mesh() -> object:
    """Timing belt loop spanning screw and motor pulleys in the YZ plane."""
    belt_x = -0.440
    cy1, cz1, r1 = 0.0, 0.100, 0.022
    cy2, cz2, r2 = -0.100, 0.055, 0.014
    n = 12

    points: list[tuple[float, float, float]] = []

    # Wrap around screw pulley (far side from motor, going CCW)
    for i in range(n):
        t = i / n
        angle = -1.078 + t * (-3.288)
        points.append((belt_x, cy1 + r1 * cos(angle), cz1 + r1 * sin(angle)))

    # Wrap around motor pulley (far side from screw, going CW)
    for i in range(n):
        t = i / n
        angle = 2.066 + t * (-3.288)
        points.append((belt_x, cy2 + r2 * cos(angle), cz2 + r2 * sin(angle)))

    return tube_from_spline_points(
        points,
        radius=0.004,
        samples_per_segment=4,
        radial_segments=8,
        closed_spline=True,
        cap_ends=False,
        up_hint=(1.0, 0.0, 0.0),
    )


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
            "run_note": (
                "Parallel/folded motor drivetrain variant: motor offset in -Y beside "
                "the rail, coupled to the lead screw via timing pulleys and reduction belt."
            ),
        },
    )

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    bright_aluminum = model.material("polished_aluminum", rgba=(0.90, 0.92, 0.88, 1.0))
    black = model.material("black_anodized", rgba=(0.015, 0.016, 0.016, 1.0))
    dark = model.material("dark_recess", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("machined_steel", rgba=(0.82, 0.83, 0.80, 1.0))
    belt_mat = model.material("black_timing_belt", rgba=(0.045, 0.045, 0.040, 1.0))
    brass = model.material("bronze_lead_nut", rgba=(0.78, 0.54, 0.24, 1.0))

    # ── Frame (root part) ──────────────────────────────────────────────
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

    # Motor block repositioned: parallel/folded layout, offset in -Y beside the rail
    frame.visual(
        mesh_from_cadquery(_motor_block(), "motor_block_mesh", tolerance=0.0008),
        origin=Origin(xyz=(-0.490, -0.100, 0.055)),
        material=black,
        name="motor_block",
    )

    # Motor bracket: flat plate connecting end plate area to the motor
    frame.visual(
        mesh_from_cadquery(_motor_bracket(), "motor_bracket_mesh", tolerance=0.0008),
        origin=Origin(xyz=(-0.445, -0.050, 0.060)),
        material=steel,
        name="motor_bracket",
    )

    # Motor pulley (fixed, on motor shaft) — CadQuery YZ workplane gives axis along X
    frame.visual(
        mesh_from_cadquery(_timing_pulley(0.014, 0.004, 0.012), "motor_pulley_mesh", tolerance=0.0005),
        origin=Origin(xyz=(-0.440, -0.100, 0.055)),
        material=steel,
        name="motor_pulley",
    )

    # Reduction belt spanning both pulleys
    frame.visual(
        mesh_from_geometry(_reduction_belt_mesh(), "reduction_belt_mesh"),
        material=belt_mat,
        name="reduction_belt",
    )

    # Drive cover panel over the belt/pulley area
    frame.visual(
        Box((0.003, 0.140, 0.080)),
        origin=Origin(xyz=(-0.450, -0.050, 0.078)),
        material=black,
        name="drive_cover",
    )

    # Connecting spacer bolts bridging bracket to drive end plate
    for idx, dy in enumerate((-0.020, 0.020)):
        frame.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(-0.437, -0.050 + dy, 0.060), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"bracket_spacer_{idx}",
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

    # Front timing-belt/rack strip on the rail side
    frame.visual(
        Box((0.730, 0.012, 0.006)),
        origin=Origin(xyz=(0.030, -0.0635, 0.012)),
        material=belt_mat,
        name="belt_backing",
    )
    tooth_count = 58
    for i in range(tooth_count):
        x = -0.330 + i * (0.660 / (tooth_count - 1))
        frame.visual(
            Box((0.0065, 0.010, 0.0045)),
            origin=Origin(xyz=(x, -0.0635, 0.01725)),
            material=belt_mat,
            name=f"belt_tooth_{i}",
        )

    # Top mounting holes on the aluminum rail
    for index, x in enumerate((-0.330, -0.210, -0.090, 0.090, 0.210, 0.330)):
        frame.visual(
            Cylinder(radius=0.0065, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.051)),
            material=dark,
            name=f"rail_hole_{index}",
        )

    # ── Carriage ───────────────────────────────────────────────────────
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
    for index, (x, y) in enumerate(((-0.042, -0.052), (0.042, -0.052), (-0.042, 0.052), (0.042, 0.052))):
        carriage.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(x, y, 0.0495)),
            material=steel,
            name=f"carriage_bolt_{index}",
        )

    # ── Lead screw ─────────────────────────────────────────────────────
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
    # Shaft extension past the drive end plate for the screw pulley
    lead_screw.visual(
        Cylinder(radius=0.0050, length=0.050),
        origin=Origin(xyz=(-0.440, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="shaft_extension",
    )
    # Screw pulley (rotates with the lead screw) — CadQuery YZ workplane gives axis along X
    lead_screw.visual(
        mesh_from_cadquery(_timing_pulley(0.022, 0.006, 0.012), "screw_pulley_mesh", tolerance=0.0005),
        origin=Origin(xyz=(-0.440, 0.0, 0.0)),
        material=steel,
        name="screw_pulley",
    )
    for index, x in enumerate((-0.420, 0.420)):
        lead_screw.visual(
            Cylinder(radius=0.0110, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"bearing_journal_{index}",
        )

    # ── Articulations ──────────────────────────────────────────────────
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

    # ── Basic structure checks ─────────────────────────────────────────
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

    # ── Parallel/folded drivetrain topology checks ─────────────────────
    ctx.check(
        "motor_bracket visual exists on frame",
        frame.get_visual("motor_bracket") is not None,
        details="Expected motor_bracket visual for parallel-drive mounting.",
    )
    ctx.check(
        "motor_pulley visual exists on frame",
        frame.get_visual("motor_pulley") is not None,
        details="Expected motor_pulley visual for reduction belt drive.",
    )
    ctx.check(
        "screw_pulley visual exists on lead_screw",
        screw.get_visual("screw_pulley") is not None,
        details="Expected screw_pulley visual on lead_screw for belt coupling.",
    )
    ctx.check(
        "reduction_belt visual exists on frame",
        frame.get_visual("reduction_belt") is not None,
        details="Expected reduction_belt visual spanning both pulleys.",
    )

    # Motor block should be offset in -Y from the rail centerline
    motor_aabb = ctx.part_element_world_aabb(frame, elem="motor_block")
    rail_aabb = ctx.part_element_world_aabb(frame, elem="base_rail")
    if motor_aabb is not None and rail_aabb is not None:
        motor_y_center = (motor_aabb[0][1] + motor_aabb[1][1]) / 2.0
        rail_y_center = (rail_aabb[0][1] + rail_aabb[1][1]) / 2.0
        ctx.check(
            "motor_block offset in -Y for parallel drive",
            motor_y_center < rail_y_center - 0.04,
            details=f"motor_y_center={motor_y_center:.4f}, rail_y_center={rail_y_center:.4f}",
        )
    else:
        ctx.fail("motor_block offset check", "Could not resolve motor or rail AABB.")

    # ── Belt / screw-pulley seating ────────────────────────────────────
    ctx.allow_overlap(
        frame,
        screw,
        elem_a="reduction_belt",
        elem_b="screw_pulley",
        reason="The reduction belt is intentionally wrapped around the screw pulley as a fixed belt-drive coupling.",
    )
    ctx.expect_overlap(
        frame,
        screw,
        axes="x",
        elem_a="reduction_belt",
        elem_b="screw_pulley",
        min_overlap=0.005,
        name="belt retained on screw pulley axially",
    )

    # ── Bearing journal / ring seating ──────────────────────────────────
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

    # ── Carriage / rail / screw fit ────────────────────────────────────
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
