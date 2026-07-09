from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


TUBE = 0.014
FOOTREST_LIFT = 0.075


def _merge_tube(
    target: MeshGeometry,
    points: list[tuple[float, float, float]],
    *,
    radius: float = TUBE,
    samples_per_segment: int = 8,
) -> None:
    """Append a round steel tube following the given local points."""
    target.merge(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=16,
            cap_ends=True,
        )
    )


def _wheel_plane_torus(major_radius: float, tube_radius: float, *, y: float = 0.0) -> MeshGeometry:
    """Torus whose axle is local Y and whose round wheel lies in local XZ."""
    return (
        TorusGeometry(major_radius, tube_radius, radial_segments=18, tubular_segments=96)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0)
    )


def _wheel_axis_cylinder(radius: float, length: float, *, y: float = 0.0) -> MeshGeometry:
    """Cylinder centered on the local Y axle."""
    return (
        CylinderGeometry(radius, length, radial_segments=40, closed=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0)
    )


def _spoke_mesh(
    *,
    hub_radius: float,
    rim_radius: float,
    count: int,
    spoke_radius: float,
    flange_y: float,
    standoff_y: float | None = None,
    push_radius: float | None = None,
) -> MeshGeometry:
    """Chrome spoke bundle; the loop below is intentional and visible."""
    geom = MeshGeometry()
    for i in range(count):
        theta = 2.0 * math.pi * i / count
        flange = flange_y if i % 2 == 0 else -flange_y
        hub_angle = theta + 0.16
        start = (
            hub_radius * math.cos(hub_angle),
            flange,
            hub_radius * math.sin(hub_angle),
        )
        end = (
            rim_radius * math.cos(theta),
            0.0,
            rim_radius * math.sin(theta),
        )
        _merge_tube(geom, [start, end], radius=spoke_radius, samples_per_segment=1)

    if standoff_y is not None and push_radius is not None:
        # Short stand-offs hold the hand push-rim proud of the wheel rim.
        for i in range(8):
            theta = 2.0 * math.pi * i / 8
            rim_point = (
                rim_radius * math.cos(theta),
                0.0,
                rim_radius * math.sin(theta),
            )
            push_point = (
                push_radius * math.cos(theta),
                standoff_y,
                push_radius * math.sin(theta),
            )
            _merge_tube(geom, [rim_point, push_point], radius=spoke_radius * 1.15, samples_per_segment=1)
    return geom


def _motor_drive_wheel_meshes() -> dict[str, MeshGeometry]:
    """Motorized drive wheel: thick tire, rim, heavy motor hub, spokes (no push-rim)."""
    return {
        "tire": _wheel_plane_torus(0.270, 0.035),
        "rim": _wheel_plane_torus(0.230, 0.008),
        "hub": _wheel_axis_cylinder(0.068, 0.095),
        "spokes": _spoke_mesh(
            hub_radius=0.065,
            rim_radius=0.226,
            count=20,
            spoke_radius=0.003,
            flange_y=0.022,
        ),
    }


def _caster_wheel_meshes() -> dict[str, MeshGeometry]:
    return {
        "tire": _wheel_plane_torus(0.079, 0.020),
        "rim": _wheel_plane_torus(0.052, 0.007),
        "hub": _wheel_axis_cylinder(0.026, 0.056),
        "spokes": _spoke_mesh(
            hub_radius=0.026,
            rim_radius=0.052,
            count=6,
            spoke_radius=0.0032,
            flange_y=0.008,
        ),
    }


def _build_frame_mesh() -> MeshGeometry:
    """One connected dark tubular wheelchair frame."""
    geom = MeshGeometry()
    for y in (-0.235, 0.235):
        # Side frame: rear upright, seat rail, front upright, lower rail, and arm support.
        _merge_tube(geom, [(-0.245, y, 0.300), (-0.245, y, 0.900), (-0.300, y, 0.960)], radius=TUBE)
        _merge_tube(geom, [(-0.245, y, 0.470), (0.300, y, 0.470)], radius=TUBE)
        _merge_tube(geom, [(0.300, y, 0.275), (0.300, y, 0.665)], radius=TUBE)
        _merge_tube(geom, [(-0.235, y, 0.310), (0.120, y, 0.265), (0.350, y, 0.245), (0.420, y, 0.240)], radius=TUBE)
        _merge_tube(geom, [(-0.245, y, 0.655), (0.300, y, 0.655)], radius=TUBE * 0.85)
        _merge_tube(geom, [(-0.220, y, 0.455), (0.300, y, 0.315)], radius=TUBE * 0.80)

        # Forward footrest hanger tube on each side.
        inner_y = 0.135 if y > 0 else -0.135
        _merge_tube(
            geom,
            [
                (0.300, y, 0.380 + FOOTREST_LIFT),
                (0.338, y, 0.360 + FOOTREST_LIFT),
                (0.405, inner_y, 0.260 + FOOTREST_LIFT),
                (0.455, inner_y, 0.201 + FOOTREST_LIFT),
            ],
            radius=TUBE * 0.82,
        )

    # Widthwise cross-members and folding X-brace below the seat.
    for x, z, radius in [
        (-0.255, 0.470, TUBE),
        (0.300, 0.470, TUBE),
        (-0.255, 0.855, TUBE * 0.82),
        (0.300, 0.300, TUBE * 0.82),
    ]:
        _merge_tube(geom, [(x, -0.235, z), (x, 0.235, z)], radius=radius, samples_per_segment=1)
    _merge_tube(geom, [(-0.210, -0.220, 0.305), (0.245, 0.220, 0.455)], radius=TUBE * 0.65)
    _merge_tube(geom, [(-0.210, 0.220, 0.305), (0.245, -0.220, 0.455)], radius=TUBE * 0.65)

    pivot = (
        CylinderGeometry(0.014, 0.018, radial_segments=24, closed=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.018, 0.0, 0.380)
    )
    geom.merge(pivot)
    return geom


def _caster_fork_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    _merge_tube(geom, [(0.0, 0.0, 0.0), (0.0, 0.0, -0.055)], radius=0.012, samples_per_segment=1)
    # Fork trails behind the vertical swivel axis so swiveling is visible.
    _merge_tube(geom, [(0.0, -0.052, -0.055), (-0.105, -0.052, -0.110)], radius=0.008, samples_per_segment=1)
    _merge_tube(geom, [(0.0, 0.052, -0.055), (-0.105, 0.052, -0.110)], radius=0.008, samples_per_segment=1)
    _merge_tube(geom, [(0.0, -0.052, -0.055), (0.0, 0.052, -0.055)], radius=0.008, samples_per_segment=1)
    _merge_tube(geom, [(-0.105, -0.058, -0.110), (-0.105, 0.058, -0.110)], radius=0.006, samples_per_segment=1)
    return geom


def _footplate_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    # A hinge barrel plus a slim angled metal footplate, all in the child frame.
    geom.merge(_wheel_axis_cylinder(0.011, 0.145))
    plate = (
        CylinderGeometry(0.001, 0.001, radial_segments=8, closed=True)
    )  # placeholder replaced by box visual in build for cleaner exact dimensions
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_wheelchair")

    steel = model.material("dark_tubular_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    fabric = model.material("blue_fabric", rgba=(0.05, 0.18, 0.36, 1.0))
    fabric_dark = model.material("blue_fabric_shadow", rgba=(0.025, 0.10, 0.23, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_build_frame_mesh(), "tubular_frame"),
        material=steel,
        name="tubular_frame",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.800),
        origin=Origin(xyz=(-0.220, 0.0, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    # Caster swivel sockets and footplate hinge pins are on the fixed frame.
    for label, y in (("left", 0.235), ("right", -0.235)):
        frame.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(0.420, y, 0.225)),
            material=steel,
            name=f"{label}_caster_socket",
        )
    for label, y in (("left", 0.135), ("right", -0.135)):
        frame.visual(
            Cylinder(radius=0.010, length=0.155),
            origin=Origin(xyz=(0.455, y, 0.180 + FOOTREST_LIFT), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{label}_footrest_hinge",
        )
        frame.visual(
            Box((0.024, 0.035, 0.020)),
            origin=Origin(xyz=(0.455, y, 0.200 + FOOTREST_LIFT)),
            material=steel,
            name=f"{label}_footrest_bracket",
        )

    # Blue upholstered seat and back; seam strips make the fabric read as padded.
    frame.visual(
        Box((0.560, 0.470, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, 0.465)),
        material=fabric,
        name="seat_cushion",
    )
    frame.visual(
        Box((0.040, 0.490, 0.430)),
        origin=Origin(xyz=(-0.275, 0.0, 0.690)),
        material=fabric,
        name="backrest",
    )
    for i, y in enumerate([v * 0.060 for v in range(-3, 4)]):
        frame.visual(
            Box((0.006, 0.010, 0.330)),
            origin=Origin(xyz=(-0.252, y, 0.690)),
            material=fabric_dark,
            name=f"backrest_seam_{i}",
        )
    for i, y in enumerate([v * 0.055 for v in range(-4, 5)]):
        frame.visual(
            Box((0.430, 0.006, 0.006)),
            origin=Origin(xyz=(0.035, y, 0.493)),
            material=fabric_dark,
            name=f"seat_seam_{i}",
        )

    # Padded side armrests on the tubular supports.
    for label, y in (("left", 0.265), ("right", -0.265)):
        frame.visual(
            Box((0.455, 0.070, 0.035)),
            origin=Origin(xyz=(0.045, y, 0.682)),
            material=fabric,
            name=f"{label}_armrest_pad",
        )

    # Dark plastic / electronics materials.
    housing_plastic = model.material("dark_housing_plastic", rgba=(0.03, 0.03, 0.035, 1.0))
    controller_grey = model.material("controller_grey", rgba=(0.12, 0.12, 0.13, 1.0))
    led_green = model.material("led_green", rgba=(0.1, 0.85, 0.2, 1.0))

    # Battery box beneath the seat, between the cross-members.
    frame.visual(
        Box((0.380, 0.340, 0.100)),
        origin=Origin(xyz=(0.010, 0.0, 0.350)),
        material=housing_plastic,
        name="battery_box",
    )
    # Battery box mounting rails (visual detail).
    for y in (-0.170, 0.170):
        frame.visual(
            Box((0.380, 0.020, 0.018)),
            origin=Origin(xyz=(0.010, y, 0.301)),
            material=steel,
            name=f"battery_rail_{'left' if y > 0 else 'right'}",
        )

    # Motor/gearbox housings flanking the rear wheels, mounted on the frame axle area.
    for label, y, side_sign in (("left", 0.310, 1.0), ("right", -0.310, -1.0)):
        frame.visual(
            Box((0.130, 0.075, 0.110)),
            origin=Origin(xyz=(-0.220, y, 0.300)),
            material=housing_plastic,
            name=f"{label}_motor_housing",
        )
        # Gearbox cap detail on outboard face.
        frame.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(-0.220, y + side_sign * 0.046, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=controller_grey,
            name=f"{label}_gearbox_cap",
        )

    # Joystick controller pod on the right armrest (forward end).
    frame.visual(
        Box((0.100, 0.080, 0.040)),
        origin=Origin(xyz=(0.240, -0.265, 0.710)),
        material=controller_grey,
        name="joystick_base",
    )
    # Joystick stem and knob.
    frame.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.240, -0.265, 0.749)),
        material=steel,
        name="joystick_stem",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.240, -0.265, 0.779)),
        material=housing_plastic,
        name="joystick_knob",
    )
    # Small LED indicator on the controller pod (embedded into top surface of base).
    frame.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.270, -0.265, 0.729)),
        material=led_green,
        name="joystick_led",
    )

    # Rear motorized drive wheels (no push-rim; motor hub replaces manual hub).
    for label, y, side_sign in (("left", 0.380, 1.0), ("right", -0.380, -1.0)):
        wheel = model.part(f"{label}_rear_wheel")
        meshes = _motor_drive_wheel_meshes()
        wheel.visual(mesh_from_geometry(meshes["tire"], f"{label}_rear_tire"), material=rubber, name="tire")
        wheel.visual(mesh_from_geometry(meshes["rim"], f"{label}_rear_rim"), material=chrome, name="rim")
        wheel.visual(mesh_from_geometry(meshes["hub"], f"{label}_rear_hub"), material=housing_plastic, name="hub")
        wheel.visual(mesh_from_geometry(meshes["spokes"], f"{label}_rear_spokes"), material=chrome, name="spokes")
        model.articulation(
            f"frame_to_{label}_rear_wheel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.220, y, 0.300)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=12.0),
        )

    # Front swivel casters, each with a trailing fork and a rolling small wheel.
    for label, y in (("left", 0.235), ("right", -0.235)):
        fork = model.part(f"{label}_caster_fork")
        fork.visual(
            mesh_from_geometry(_caster_fork_mesh(), f"{label}_caster_fork"),
            material=chrome,
            name="fork",
        )
        model.articulation(
            f"frame_to_{label}_caster",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=fork,
            origin=Origin(xyz=(0.420, y, 0.210)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

        caster_wheel = model.part(f"{label}_caster_wheel")
        caster_meshes = _caster_wheel_meshes()
        caster_wheel.visual(mesh_from_geometry(caster_meshes["tire"], f"{label}_caster_tire"), material=rubber, name="tire")
        caster_wheel.visual(mesh_from_geometry(caster_meshes["rim"], f"{label}_caster_rim"), material=chrome, name="rim")
        caster_wheel.visual(mesh_from_geometry(caster_meshes["hub"], f"{label}_caster_hub"), material=chrome, name="hub")
        caster_wheel.visual(mesh_from_geometry(caster_meshes["spokes"], f"{label}_caster_spokes"), material=chrome, name="spokes")
        model.articulation(
            f"{label}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=caster_wheel,
            origin=Origin(xyz=(-0.105, 0.0, -0.110)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    # Flip-up angled footplates.
    for label, y in (("left", 0.135), ("right", -0.135)):
        footplate = model.part(f"{label}_footplate")
        footplate.visual(
            mesh_from_geometry(_footplate_mesh(), f"{label}_footplate_hinge"),
            material=chrome,
            name="hinge_bar",
        )
        footplate.visual(
            Box((0.180, 0.120, 0.014)),
            origin=Origin(xyz=(0.090, 0.0, -0.026), rpy=(0.0, -0.18, 0.0)),
            material=steel,
            name="plate",
        )
        footplate.visual(
            Box((0.110, 0.118, 0.036)),
            origin=Origin(xyz=(0.045, 0.0, -0.018), rpy=(0.0, -0.08, 0.0)),
            material=steel,
            name="plate_neck",
        )
        model.articulation(
            f"frame_to_{label}_footplate",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=footplate,
            origin=Origin(xyz=(0.455, y, 0.180 + FOOTREST_LIFT)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    for label in ("left", "right"):
        rear_wheel = object_model.get_part(f"{label}_rear_wheel")
        caster_fork = object_model.get_part(f"{label}_caster_fork")
        caster_wheel = object_model.get_part(f"{label}_caster_wheel")
        footplate = object_model.get_part(f"{label}_footplate")

        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a="rear_axle",
            elem_b="hub",
            reason="The fixed rear axle intentionally passes through the motor drive wheel hub bearing.",
        )
        ctx.expect_overlap(
            frame,
            rear_wheel,
            axes="y",
            elem_a="rear_axle",
            elem_b="hub",
            min_overlap=0.030,
            name=f"{label} rear motor hub is retained on the axle",
        )
        ctx.expect_overlap(
            frame,
            rear_wheel,
            axes="xz",
            elem_a="rear_axle",
            elem_b="hub",
            min_overlap=0.030,
            name=f"{label} rear motor hub surrounds the axle line",
        )

        # Motor housing wraps partially around the wheel hub (intentional local embedding).
        ctx.allow_overlap(
            frame,
            rear_wheel,
            elem_a=f"{label}_motor_housing",
            elem_b="hub",
            reason=f"The {label} motor/gearbox housing intentionally wraps around the inboard side of the drive wheel motor hub.",
        )
        # Motor housing must be near the corresponding drive wheel on XZ axes.
        ctx.expect_overlap(
            frame,
            rear_wheel,
            axes="xz",
            elem_a=f"{label}_motor_housing",
            elem_b="tire",
            min_overlap=0.040,
            name=f"{label} motor housing is co-located with the drive wheel",
        )
        ctx.expect_contact(
            frame,
            rear_wheel,
            elem_a=f"{label}_motor_housing",
            elem_b="hub",
            name=f"{label} motor housing contacts the drive hub",
        )

        ctx.allow_overlap(
            caster_fork,
            caster_wheel,
            elem_a="fork",
            elem_b="hub",
            reason="The caster fork axle pin is intentionally captured through the small wheel hub.",
        )
        ctx.expect_overlap(
            caster_fork,
            caster_wheel,
            axes="y",
            elem_a="fork",
            elem_b="hub",
            min_overlap=0.030,
            name=f"{label} caster wheel is captured by fork axle",
        )

        ctx.allow_overlap(
            frame,
            footplate,
            elem_a=f"{label}_footrest_hinge",
            elem_b="hinge_bar",
            reason="The flip-up footplate hinge barrel is intentionally coaxial with the frame hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            footplate,
            axes="y",
            elem_a=f"{label}_footrest_hinge",
            elem_b="hinge_bar",
            min_overlap=0.060,
            name=f"{label} footplate hinge is pinned to the frame",
        )
        ctx.allow_overlap(
            frame,
            footplate,
            elem_a=f"{label}_footrest_hinge",
            elem_b="plate_neck",
            reason="A small welded hinge lug wraps around the same footplate hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            footplate,
            axes="y",
            elem_a=f"{label}_footrest_hinge",
            elem_b="plate_neck",
            min_overlap=0.050,
            name=f"{label} footplate lug wraps the hinge pin",
        )

    seat_aabb = ctx.part_element_world_aabb(frame, elem="seat_cushion")
    ctx.check(
        "seat cushion is at wheelchair sitting height",
        seat_aabb is not None and 0.43 <= (seat_aabb[0][2] + seat_aabb[1][2]) / 2.0 <= 0.50,
        details=f"seat_aabb={seat_aabb}",
    )

    left_rear_aabb = ctx.part_world_aabb("left_rear_wheel")
    ctx.check(
        "rear drive wheel diameter is about 0.6 m",
        left_rear_aabb is not None and 0.56 <= (left_rear_aabb[1][2] - left_rear_aabb[0][2]) <= 0.64,
        details=f"left_rear_aabb={left_rear_aabb}",
    )

    left_caster_aabb = ctx.part_world_aabb("left_caster_wheel")
    ctx.check(
        "front caster wheel diameter is about 0.2 m",
        left_caster_aabb is not None and 0.17 <= (left_caster_aabb[1][2] - left_caster_aabb[0][2]) <= 0.22,
        details=f"left_caster_aabb={left_caster_aabb}",
    )

    # Battery box must exist beneath the seat and have meaningful size.
    battery_aabb = ctx.part_element_world_aabb(frame, elem="battery_box")
    ctx.check(
        "battery box is present beneath the seat",
        battery_aabb is not None
        and (battery_aabb[1][2] - battery_aabb[0][2]) > 0.050
        and battery_aabb[1][2] < 0.44,
        details=f"battery_aabb={battery_aabb}",
    )

    # Joystick controller pod must be present on the right armrest.
    joystick_aabb = ctx.part_element_world_aabb(frame, elem="joystick_base")
    ctx.check(
        "joystick controller base is present on the right armrest",
        joystick_aabb is not None
        and joystick_aabb[0][2] > 0.680,
        details=f"joystick_aabb={joystick_aabb}",
    )

    left_foot_joint = object_model.get_articulation("frame_to_left_footplate")
    rest_aabb = ctx.part_world_aabb("left_footplate")
    with ctx.pose({left_foot_joint: 1.25}):
        flipped_aabb = ctx.part_world_aabb("left_footplate")
    ctx.check(
        "left footplate flips upward",
        rest_aabb is not None
        and flipped_aabb is not None
        and flipped_aabb[1][2] > rest_aabb[1][2] + 0.050,
        details=f"rest={rest_aabb}, flipped={flipped_aabb}",
    )

    left_swivel = object_model.get_articulation("frame_to_left_caster")
    caster_rest = ctx.part_world_position("left_caster_wheel")
    with ctx.pose({left_swivel: math.pi / 2.0}):
        caster_turned = ctx.part_world_position("left_caster_wheel")
    ctx.check(
        "left caster swivels around a vertical stem with visible trail",
        caster_rest is not None
        and caster_turned is not None
        and abs(caster_turned[1] - caster_rest[1]) > 0.025,
        details=f"rest={caster_rest}, turned={caster_turned}",
    )

    # Drive wheels still spin: verify the continuous joint exists and is CONTINUOUS.
    left_drive = object_model.get_articulation("frame_to_left_rear_wheel")
    ctx.check(
        "left rear drive wheel joint is CONTINUOUS (motorized spin)",
        left_drive.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={left_drive.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
