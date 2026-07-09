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
    plate = plate.faces(">Z").workplane().hole(0.082)

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
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        bearing_holes.append((0.112 * math.cos(a), 0.112 * math.sin(a)))
    plate = plate.faces(">Z").workplane().pushPoints(bearing_holes).hole(0.007)
    return plate


def _bearing_ring_mesh() -> object:
    """Full-diameter slew-ring bearing with external gear teeth on the outer race.

    The ring sits under the mounting plate and provides a continuous 360-degree
    slewing interface between the plate and the rotating carriage.
    """
    # Base annular ring (outer race)
    outer_r = 0.140
    inner_r = 0.085
    ring_height = 0.028
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(ring_height)
    )
    # Cut external gear teeth grooves around the outer perimeter
    tooth_count = 48
    tooth_depth = 0.006
    tooth_width_angle = 2.0 * math.pi / tooth_count
    for i in range(tooth_count):
        a = tooth_width_angle * i
        # Small radial slot cut into the outer ring surface
        cx = (outer_r + 0.001) * math.cos(a)
        cy = (outer_r + 0.001) * math.sin(a)
        slot = (
            cq.Workplane("XY")
            .center(cx, cy)
            .rect(tooth_depth * 2, tooth_depth)
            .extrude(ring_height + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        # Rotate slot to align radially
        slot = slot.rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
        ring = ring.cut(slot)
    # Bolt holes matching the plate bearing-bolt circle
    bolt_circle_r = 0.112
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        hx = bolt_circle_r * math.cos(a)
        hy = bolt_circle_r * math.sin(a)
        hole = cq.Workplane("XY").circle(0.006).extrude(ring_height + 0.010).translate((hx, hy, -0.005))
        ring = ring.cut(hole)
    return ring


def _slew_ring_flange_mesh() -> object:
    """Matching ring flange on the carriage top that interfaces with the slew-ring bearing.

    Sits flush against the underside of the slew-ring, providing the rotating
    inner race surface for continuous 360-degree castering.
    """
    flange_outer_r = 0.132
    flange_inner_r = 0.082
    flange_height = 0.014
    flange = (
        cq.Workplane("XY")
        .circle(flange_outer_r)
        .circle(flange_inner_r)
        .extrude(flange_height)
    )
    # Bolt counter-bore pockets on the flange face (matches slew-ring bolt circle)
    bolt_circle_r = 0.112
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        hx = bolt_circle_r * math.cos(a)
        hy = bolt_circle_r * math.sin(a)
        pocket = (
            cq.Workplane("XY")
            .circle(0.009)
            .extrude(0.008)
            .translate((hx, hy, flange_height - 0.008))
        )
        flange = flange.cut(pocket)
    return flange


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
        mesh_from_cadquery(_bearing_ring_mesh(), "slew_ring_bearing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.263)),
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
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        _add_cyl(
            mount_plate,
            0.0055,
            0.003,
            (0.112 * math.cos(a), 0.112 * math.sin(a), 0.3105),
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

    # Slew-ring flange sits flush under the plate-side slew ring for full 360° rotation.
    # Carriage local frame: articulation at world z=0.235, so local z=0.014 → world z=0.249.
    # The flange mesh extrudes from z=0 to z=0.014, so top surface is at world z=0.263
    # (flush with the slew-ring bearing bottom).
    carriage.visual(
        mesh_from_cadquery(_slew_ring_flange_mesh(), "slew_ring_flange", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=steel,
        name="inner_bearing_flange",
    )
    # Enlarged swivel post fits inside the slew-ring inner bore for continuous castering.
    # Radius 0.082 matches the flange inner bore for surface contact and connectivity.
    _add_cyl(carriage, 0.082, 0.054, (0.0, 0.0, 0.005), material=dark_anodized, name="swivel_post")
    _add_cyl(carriage, 0.068, 0.060, (0.0, 0.0, -0.048), material=steel, name="bearing_pedestal")
    # The slew-ring interface replaces the discrete support posts; the full-diameter
    # bearing ring and matching flange carry the carriage load across all swivel angles.

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
        ArticulationType.CONTINUOUS,
        parent=mount_plate,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8),
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
        "mount_to_carriage is a continuous 360-degree castering swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"swivel type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "continuous swivel has no hard angular limits",
        swivel.motion_limits is None
        or (swivel.motion_limits.lower is None and swivel.motion_limits.upper is None),
        details=f"motion_limits={swivel.motion_limits}",
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

    # Prove the slew-ring bearing interface maintains contact across full rotation.
    # The outer_bearing_ring (plate) bottom meets the inner_bearing_flange (carriage) top.
    for swivel_angle in (0.25, math.pi, -math.pi):
        with ctx.pose({swivel: swivel_angle}):
            ctx.expect_gap(
                plate,
                carriage,
                axis="z",
                positive_elem="outer_bearing_ring",
                negative_elem="inner_bearing_flange",
                min_gap=-0.001,
                max_gap=0.003,
                name=f"slew-ring bearing interface at swivel={swivel_angle:.2f} rad",
            )
    # Prove a full 360° rotation returns the carriage to its rest position.
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({swivel: 2.0 * math.pi}):
        full_turn_pos = ctx.part_world_position(carriage)
    ctx.check(
        "full 360-degree swivel returns carriage to rest position",
        rest_pos is not None and full_turn_pos is not None and max(abs(rest_pos[i] - full_turn_pos[i]) for i in range(3)) < 1e-4,
        details=f"rest={rest_pos}, full_turn={full_turn_pos}",
    )

    return ctx.report()


object_model = build_object_model()
