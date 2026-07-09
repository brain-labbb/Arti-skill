from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mounting_plate_mesh() -> object:
    """Rounded top plate with a central access hole and bolt pattern."""
    plate = (
        cq.Workplane("XY")
        .box(0.620, 0.430, 0.018)
        .edges("|Z")
        .fillet(0.035)
    )
    plate = plate.faces(">Z").workplane().hole(0.105)

    perimeter_holes = [
        (-0.265, -0.165),
        (0.265, -0.165),
        (-0.265, 0.165),
        (0.265, 0.165),
        (-0.160, -0.185),
        (0.160, -0.185),
        (-0.160, 0.185),
        (0.160, 0.185),
    ]
    plate = plate.faces(">Z").workplane().pushPoints(perimeter_holes).hole(0.012)

    bearing_holes = []
    for i in range(10):
        a = 2.0 * math.pi * i / 10.0
        bearing_holes.append((0.078 * math.cos(a), 0.078 * math.sin(a)))
    plate = plate.faces(">Z").workplane().pushPoints(bearing_holes).hole(0.007)
    return plate


def _bearing_ring_mesh() -> object:
    """Thin annular ring under the mounting plate around the swivel opening."""
    outer = cq.Workplane("XY").circle(0.090).extrude(0.024, both=True)
    inner_cut = cq.Workplane("XY").circle(0.052).extrude(0.030, both=True)
    return outer.cut(inner_cut)


def _eye_bolt_mesh() -> object:
    """Small vertical lift eye with a threaded stem, as seen on the plate."""
    ring = (
        cq.Workplane("XZ")
        .circle(0.014)
        .circle(0.008)
        .extrude(0.005, both=True)
        .translate((0.0, 0.0, 0.036))
    )
    stem = cq.Workplane("XY").circle(0.003).extrude(0.028)
    base = cq.Workplane("XY").circle(0.007).extrude(0.004)
    return ring.union(stem).union(base)


def _make_tire_mesh(name: str) -> object:
    tire = TireGeometry(
        0.132,
        0.078,
        inner_radius=0.099,
        carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.030),
        tread=TireTread(style="block", depth=0.0045, count=28, land_ratio=0.62),
        grooves=(
            TireGroove(center_offset=-0.018, width=0.0045, depth=0.0025),
            TireGroove(center_offset=0.018, width=0.0045, depth=0.0025),
        ),
        sidewall=TireSidewall(style="square", bulge=0.018),
        shoulder=TireShoulder(width=0.007, radius=0.003),
    )
    return mesh_from_geometry(tire, f"{name}_polyurethane_tire")


def _add_cyl(part, radius: float, length: float, xyz, *, rpy=(0.0, 0.0, 0.0), material=None, name=None) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="differential_drive_wheel_module",
        meta={
            "category": "Robotics / Differential drive wheel",
            "reference_notes": "Two tan treaded drive wheels on a shared axle line with a metal top mounting plate, central swivel bearing, gearbox carriage, motor can, brackets, and fasteners.",
        },
    )

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.085, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    tan_poly = model.material("tan_polyurethane", rgba=(0.91, 0.62, 0.25, 1.0))
    steel = model.material("satin_steel", rgba=(0.50, 0.52, 0.51, 1.0))

    # Root: the plate that would bolt the module into a robot chassis.
    mount_plate = model.part("mount_plate")
    mount_plate.visual(
        mesh_from_cadquery(_mounting_plate_mesh(), "rounded_mount_plate", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=brushed_aluminum,
        name="rounded_plate",
    )
    mount_plate.visual(
        mesh_from_cadquery(_bearing_ring_mesh(), "outer_swivel_bearing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.277)),
        material=dark_anodized,
        name="outer_bearing_ring",
    )

    # Flush fasteners on the top plate and the bearing circle.
    for i, (x, y) in enumerate(
        [
            (-0.265, -0.165),
            (0.265, -0.165),
            (-0.265, 0.165),
            (0.265, 0.165),
            (-0.160, -0.185),
            (0.160, -0.185),
            (-0.160, 0.185),
            (0.160, 0.185),
        ]
    ):
        _add_cyl(mount_plate, 0.010, 0.004, (x, y, 0.311), material=steel, name=f"plate_screw_{i}")
    for i in range(10):
        a = 2.0 * math.pi * i / 10.0
        _add_cyl(
            mount_plate,
            0.0055,
            0.003,
            (0.078 * math.cos(a), 0.078 * math.sin(a), 0.3105),
            material=steel,
            name=f"bearing_screw_{i}",
        )

    for i, (x, y, yaw) in enumerate([(-0.095, 0.075, 0.0), (0.105, 0.075, 0.45), (0.000, -0.105, -0.35)]):
        mount_plate.visual(
            mesh_from_cadquery(_eye_bolt_mesh(), f"lift_eye_{i}", tolerance=0.0006),
            origin=Origin(xyz=(x, y, 0.309), rpy=(0.0, 0.0, yaw)),
            material=dark_anodized,
            name=f"lift_eye_{i}",
        )

    # Carriage that turns under the top plate: gearbox blocks, motor and axle supports.
    carriage = model.part("drive_carriage")
    carriage.visual(Box((0.300, 0.135, 0.078)), origin=Origin(xyz=(0.0, -0.010, -0.108)), material=brushed_aluminum, name="center_gearbox")
    carriage.visual(Box((0.105, 0.155, 0.120)), origin=Origin(xyz=(-0.180, -0.012, -0.112)), material=brushed_aluminum, name="side_gearbox_0")
    carriage.visual(Box((0.105, 0.155, 0.120)), origin=Origin(xyz=(0.180, -0.012, -0.112)), material=brushed_aluminum, name="side_gearbox_1")
    carriage.visual(Box((0.175, 0.030, 0.125)), origin=Origin(xyz=(-0.105, 0.070, -0.112)), material=steel, name="front_yoke_0")
    carriage.visual(Box((0.175, 0.030, 0.125)), origin=Origin(xyz=(0.105, 0.070, -0.112)), material=steel, name="front_yoke_1")
    carriage.visual(Box((0.255, 0.034, 0.035)), origin=Origin(xyz=(0.0, 0.076, -0.046)), material=steel, name="upper_cross_brace")

    # Bearing post fits up into the annular bearing without being a solid overlap.
    _add_cyl(carriage, 0.049, 0.050, (0.0, 0.0, 0.006), material=dark_anodized, name="swivel_post")
    _add_cyl(carriage, 0.066, 0.012, (0.0, 0.0, -0.023), material=steel, name="inner_bearing_flange")
    _add_cyl(carriage, 0.055, 0.055, (0.0, 0.0, -0.045), material=steel, name="bearing_pedestal")
    for i, (x, y) in enumerate([(-0.120, 0.0), (0.120, 0.0), (0.0, -0.120)]):
        _add_cyl(carriage, 0.010, 0.120, (x, y, -0.014), material=steel, name=f"top_support_post_{i}")
        _add_cyl(carriage, 0.014, 0.020, (x, y, 0.046), material=steel, name=f"top_support_pad_{i}")

    # Two compact motor cans are coaxial with the axle line and braced by the gearbox blocks.
    motor_rpy = (0.0, math.pi / 2.0, 0.0)
    _add_cyl(carriage, 0.047, 0.205, (0.0, -0.107, -0.106), rpy=motor_rpy, material=dark_anodized, name="motor_can")
    _add_cyl(carriage, 0.050, 0.010, (-0.108, -0.107, -0.106), rpy=motor_rpy, material=steel, name="motor_endcap_0")
    _add_cyl(carriage, 0.050, 0.010, (0.108, -0.107, -0.106), rpy=motor_rpy, material=steel, name="motor_endcap_1")
    for i, z in enumerate([-0.128, -0.106, -0.084]):
        carriage.visual(Box((0.226, 0.007, 0.007)), origin=Origin(xyz=(0.0, -0.156, z)), material=black_rubber, name=f"motor_fin_{i}")

    # Axle stubs are visible but stop just shy of the rotating wheel hubs.
    _add_cyl(carriage, 0.014, 0.031, (-0.237, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_stub_0")
    _add_cyl(carriage, 0.014, 0.031, (0.237, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_stub_1")
    _add_cyl(carriage, 0.024, 0.014, (-0.220, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_bearing_0")
    _add_cyl(carriage, 0.024, 0.014, (0.220, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_bearing_1")

    # Side face socket screws on the gearbox covers.
    for side, x_face in enumerate([-0.2325, 0.2325]):
        sign = -1.0 if x_face < 0 else 1.0
        for j, (y, z) in enumerate([(-0.055, -0.070), (0.045, -0.070), (-0.055, -0.145), (0.045, -0.145)]):
            _add_cyl(
                carriage,
                0.006,
                0.004,
                (x_face + sign * 0.002, y, z),
                rpy=motor_rpy,
                material=dark_anodized,
                name=f"gearbox_screw_{side}_{j}",
            )

    # Rotating wheels: tan treaded tires, machined hubs and small outer retaining caps.
    wheel_positions = [(-0.294, 0.0, -0.105), (0.294, 0.0, -0.105)]
    wheels = []
    for i, pos in enumerate(wheel_positions):
        wheel_part = model.part(f"wheel_{i}")
        tire_mesh = _make_tire_mesh(f"wheel_{i}")
        wheel_part.visual(tire_mesh, material=tan_poly, name="treaded_tire")
        _add_cyl(wheel_part, 0.100, 0.073, (0.0, 0.0, 0.0), rpy=motor_rpy, material=brushed_aluminum, name="rim_shell")
        _add_cyl(wheel_part, 0.041, 0.083, (0.0, 0.0, 0.0), rpy=motor_rpy, material=steel, name="hub_drum")
        cap_x = -0.0395 if i == 0 else 0.0395
        _add_cyl(wheel_part, 0.036, 0.006, (cap_x, 0.0, 0.0), rpy=motor_rpy, material=steel, name="outer_hub_cap")
        screw_x = -0.044 if i == 0 else 0.044
        for j in range(10):
            a = 2.0 * math.pi * j / 10.0
            _add_cyl(
                wheel_part,
                0.0032,
                0.003,
                (screw_x, 0.026 * math.cos(a), 0.026 * math.sin(a)),
                rpy=motor_rpy,
                material=dark_anodized,
                name=f"hub_screw_{j}",
            )
        wheels.append(wheel_part)

    # Articulations: visible top swivel and two drive-wheel rotations.
    model.articulation(
        "mount_to_carriage",
        ArticulationType.REVOLUTE,
        parent=mount_plate,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.35, upper=0.35),
        motion_properties=MotionProperties(damping=0.3, friction=0.1),
    )

    for i, (wheel_part, pos) in enumerate(zip(wheels, wheel_positions)):
        model.articulation(
            f"carriage_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel_part,
            origin=Origin(xyz=pos),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("mount_plate")
    carriage = object_model.get_part("drive_carriage")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    swivel = object_model.get_articulation("mount_to_carriage")
    spin_0 = object_model.get_articulation("carriage_to_wheel_0")
    spin_1 = object_model.get_articulation("carriage_to_wheel_1")

    ctx.check(
        "reference category matches visible differential drive module",
        True,
        details="No classification mismatch suspected: image and folder both indicate a robotics differential drive wheel module.",
    )

    ctx.check(
        "module has two independent rotating wheels",
        spin_0.articulation_type == ArticulationType.CONTINUOUS and spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"spin types: {spin_0.articulation_type}, {spin_1.articulation_type}",
    )
    ctx.check("wheel axes are collinear", spin_0.axis == (1.0, 0.0, 0.0) and spin_1.axis == (1.0, 0.0, 0.0), details=f"axes: {spin_0.axis}, {spin_1.axis}")
    ctx.check(
        "top bearing is a limited steering swivel",
        swivel.articulation_type == ArticulationType.REVOLUTE and swivel.motion_limits is not None and swivel.motion_limits.lower < 0.0 < swivel.motion_limits.upper,
        details=f"swivel={swivel}",
    )

    # Wheels sit outboard of the gearbox with a small functional clearance, while
    # still overlapping the carriage footprint in Y/Z so they read as axle-mounted.
    ctx.expect_gap(
        carriage,
        wheel_0,
        axis="x",
        negative_elem="treaded_tire",
        min_gap=0.001,
        max_gap=0.060,
        name="wheel 0 tire clears the gearbox side",
    )
    ctx.expect_gap(
        wheel_1,
        carriage,
        axis="x",
        positive_elem="treaded_tire",
        min_gap=0.001,
        max_gap=0.060,
        name="wheel 1 tire clears the gearbox side",
    )
    ctx.expect_overlap(wheel_0, carriage, axes="yz", min_overlap=0.080, name="wheel 0 aligns to axle supports")
    ctx.expect_overlap(wheel_1, carriage, axes="yz", min_overlap=0.080, name="wheel 1 aligns to axle supports")

    rest_0 = ctx.part_world_position(wheel_0)
    rest_1 = ctx.part_world_position(wheel_1)
    with ctx.pose({spin_0: 1.2, spin_1: -1.2}):
        spun_0 = ctx.part_world_position(wheel_0)
        spun_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheel spin keeps axle centers fixed",
        rest_0 is not None and spun_0 is not None and rest_1 is not None and spun_1 is not None and max(abs(rest_0[i] - spun_0[i]) for i in range(3)) < 1e-6 and max(abs(rest_1[i] - spun_1[i]) for i in range(3)) < 1e-6,
        details=f"rest=({rest_0}, {rest_1}) spun=({spun_0}, {spun_1})",
    )

    with ctx.pose({swivel: 0.25}):
        ctx.expect_gap(
            plate,
            carriage,
            axis="z",
            positive_elem="rounded_plate",
            min_gap=0.0,
            max_gap=0.002,
            name="swiveled support pads ride under the solid top plate",
        )

    return ctx.report()


object_model = build_object_model()
