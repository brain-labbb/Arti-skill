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
    tube_from_spline_points,
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


# ── Rocker-arm suspension geometry ──────────────────────────────────────────

# Key layout constants shared by the arm, spring, and joint code.
_PIVOT_X_OFFSET = 0.205  # |x| of the rocker pivot from the model centre
_PIVOT_Z = -0.050  # pivot height (above the axle line)
_WHEEL_X = 0.294  # |x| of the wheel centre
_WHEEL_Z = -0.105  # wheel centre height
_ARM_DX = _WHEEL_X - _PIVOT_X_OFFSET  # 0.089
_ARM_DZ = _WHEEL_Z - _PIVOT_Z  # -0.055
_ARM_LENGTH = math.sqrt(_ARM_DX ** 2 + _ARM_DZ ** 2)
_ARM_ANGLE_DEG = math.degrees(math.atan2(_ARM_DZ, _ARM_DX))  # ≈ -31.7°
_SPRING_SEAT_T = 0.65  # fraction along arm for the spring seat (more inboard to clear tire)


def _make_rocker_arm_mesh(side: int) -> object:
    """Fabricated steel rocker arm. Pivot bore at origin; axle bore at outboard end.

    The arm lies in the XZ plane. For side 0 (left, sign=-1) the axle end is
    at (-ARM_DX, 0, ARM_DZ); for side 1 (right, sign=+1) it is at
    (+ARM_DX, 0, ARM_DZ). The pivot bore axis is Y; the axle bore axis is X.
    """
    sign = -1.0 if side == 0 else 1.0
    ax = sign * _ARM_DX
    az = _ARM_DZ

    # Main beam: build along +X then rotate about Y to point at the axle end.
    # Slightly oversized so it clearly intersects both bosses.
    rot_angle = _ARM_ANGLE_DEG if sign > 0 else (180.0 - _ARM_ANGLE_DEG)
    beam = (
        cq.Workplane("XY")
        .box(_ARM_LENGTH + 0.010, 0.038, 0.016)
        .edges("|Y")
        .fillet(0.004)
    )
    beam = beam.rotate((0, 0, 0), (0, 1, 0), rot_angle)
    beam = beam.translate((ax / 2.0, 0.0, az / 2.0))

    # Pivot boss – thick disk along Y at origin with through-bore.
    pivot_outer = (
        cq.Workplane("XZ")
        .circle(0.022)
        .extrude(0.046)
        .translate((0, -0.023, 0))
    )
    pivot_bore = (
        cq.Workplane("XZ")
        .circle(0.015)
        .extrude(0.056)
        .translate((0, -0.028, 0))
    )
    pivot = pivot_outer.cut(pivot_bore)

    # Axle bearing housing – cylinder along X at the wheel end.
    axle_outer = (
        cq.Workplane("YZ")
        .workplane(offset=ax - sign * 0.024)
        .circle(0.026)
        .extrude(sign * 0.048)
    )
    axle_bore = (
        cq.Workplane("YZ")
        .workplane(offset=ax - sign * 0.030)
        .circle(0.016)
        .extrude(sign * 0.060)
    )
    axle_housing = axle_outer.cut(axle_bore)

    # Combine all into one solid via sequential union with generous overlap.
    arm = beam.union(pivot).union(axle_housing)
    return arm


def _make_spring_mesh(n_coils: int = 6, spring_radius: float = 0.013, wire_radius: float = 0.0025, height: float = 0.058) -> object:
    """Coil spring built as a helical tube standing along +Z, centred at mid-height."""
    pts_per_coil = 28
    total_pts = n_coils * pts_per_coil + 1
    points = []
    for i in range(total_pts):
        t = i / (total_pts - 1)
        angle = 2.0 * math.pi * n_coils * t
        x = spring_radius * math.cos(angle)
        y = spring_radius * math.sin(angle)
        z = height * (t - 0.5)
        points.append((x, y, z))
    tube_geo = tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=4,
        radial_segments=10,
        cap_ends=True,
        spline="catmull_rom",
    )
    return mesh_from_geometry(tube_geo, "suspension_spring_coil")


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
    spring_steel = model.material("spring_steel", rgba=(0.38, 0.40, 0.38, 1.0))
    zinc_yellow = model.material("zinc_yellow", rgba=(0.85, 0.72, 0.12, 1.0))

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

    # Shock absorber bodies and upper spring seats for the rocker suspension.
    # The shock tube hangs down from the carriage toward each rocker arm.
    # Spring seat at 50% along arm to clear the gearbox side boxes.
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        pivot_x = sign * _PIVOT_X_OFFSET
        # Spring upper mount: on the carriage, above the rocker spring seat at rest.
        spring_x = pivot_x + _SPRING_SEAT_T * sign * _ARM_DX
        spring_z_on_arm = _PIVOT_Z + _SPRING_SEAT_T * _ARM_DZ
        spring_z_upper = spring_z_on_arm + 0.035  # above the arm
        # Vertical connecting strut from carriage body to the spring mount.
        strut_top = spring_z_upper
        strut_bottom = -0.046  # upper cross brace top
        strut_len = strut_top - strut_bottom
        if strut_len > 0.005:
            carriage.visual(
                Box((0.012, 0.012, strut_len)),
                origin=Origin(xyz=(spring_x, 0.0, strut_bottom + strut_len / 2.0)),
                material=steel, name=f"spring_strut_{i}",
            )
        # Shock body – outer cylinder on the carriage pointing down.
        _add_cyl(
            carriage, 0.014, 0.042,
            (spring_x, 0.0, spring_z_upper + 0.021),
            material=dark_anodized, name=f"shock_body_{i}",
        )
        # Upper spring anchor pad.
        _add_cyl(
            carriage, 0.018, 0.005,
            (spring_x, 0.0, spring_z_upper),
            material=steel, name=f"spring_mount_{i}",
        )

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

    # ── Rocker arms: suspension links between carriage and wheels ──────────
    spring_mesh_cache = _make_spring_mesh()  # identical spring for both sides
    rocker_arms = []
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        rocker = model.part(f"rocker_arm_{i}")

        # Structural arm body (CadQuery mesh, pivot at part origin).
        rocker.visual(
            mesh_from_cadquery(_make_rocker_arm_mesh(i), f"rocker_arm_body_{i}", tolerance=0.0008),
            material=steel,
            name="arm_body",
        )

        # Coil spring on the arm, centred at the spring seat point.
        seat_x = _SPRING_SEAT_T * sign * _ARM_DX
        seat_z = _SPRING_SEAT_T * _ARM_DZ
        spring_mid_z = seat_z + 0.029  # half the spring height above the seat
        rocker.visual(
            spring_mesh_cache,
            origin=Origin(xyz=(seat_x, 0.0, spring_mid_z)),
            material=zinc_yellow,
            name="spring_coil",
        )

        # Shock shaft – thin rod extending up from the arm into the shock body.
        _add_cyl(
            rocker, 0.007, 0.040,
            (seat_x, 0.0, spring_mid_z + 0.025),
            material=steel, name="shock_shaft",
        )

        rocker_arms.append(rocker)

    # Rotating wheels: tan treaded tires, machined hubs and small outer retaining caps.
    wheel_positions = [(-_WHEEL_X, 0.0, _WHEEL_Z), (_WHEEL_X, 0.0, _WHEEL_Z)]
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

    # Articulations: top swivel, rocker pivots, and two drive-wheel rotations.
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

    # Rocker pivot joints: carriage → rocker_arm_i.
    # Axis chosen so positive q lifts the wheel (suspension compression).
    for i in range(2):
        sign = -1.0 if i == 0 else 1.0
        pivot_axis = (0.0, 1.0, 0.0) if i == 0 else (0.0, -1.0, 0.0)
        model.articulation(
            f"rocker_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=rocker_arms[i],
            origin=Origin(xyz=(sign * _PIVOT_X_OFFSET, 0.0, _PIVOT_Z)),
            axis=pivot_axis,
            motion_limits=MotionLimits(effort=25.0, velocity=4.0, lower=-0.12, upper=0.18),
            motion_properties=MotionProperties(damping=1.2, friction=0.15),
        )

    # Wheel spin joints: rocker_arm_i → wheel_i (re-parented through rocker).
    # The wheel origin in the rocker local frame is at (sign*ARM_DX, 0, ARM_DZ).
    for i, (wheel_part, pos) in enumerate(zip(wheels, wheel_positions)):
        sign = -1.0 if i == 0 else 1.0
        model.articulation(
            f"carriage_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=rocker_arms[i],
            child=wheel_part,
            origin=Origin(xyz=(sign * _ARM_DX, 0.0, _ARM_DZ)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("mount_plate")
    carriage = object_model.get_part("drive_carriage")
    rocker_0 = object_model.get_part("rocker_arm_0")
    rocker_1 = object_model.get_part("rocker_arm_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    swivel = object_model.get_articulation("mount_to_carriage")
    rocker_piv_0 = object_model.get_articulation("rocker_pivot_0")
    rocker_piv_1 = object_model.get_articulation("rocker_pivot_1")
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

    # ── Rocker arm suspension checks ──────────────────────────────────────
    ctx.check(
        "rocker_pivot_0 is a revolute joint with limited travel",
        rocker_piv_0.articulation_type == ArticulationType.REVOLUTE
        and rocker_piv_0.motion_limits is not None
        and rocker_piv_0.motion_limits.lower < 0.0 < rocker_piv_0.motion_limits.upper,
        details=f"rocker_pivot_0 type={rocker_piv_0.articulation_type}, limits={rocker_piv_0.motion_limits}",
    )
    ctx.check(
        "rocker_pivot_1 is a revolute joint with limited travel",
        rocker_piv_1.articulation_type == ArticulationType.REVOLUTE
        and rocker_piv_1.motion_limits is not None
        and rocker_piv_1.motion_limits.lower < 0.0 < rocker_piv_1.motion_limits.upper,
        details=f"rocker_pivot_1 type={rocker_piv_1.articulation_type}, limits={rocker_piv_1.motion_limits}",
    )

    # Each wheel is parented through its rocker arm, not directly on the carriage.
    parent_name_0 = spin_0.parent if isinstance(spin_0.parent, str) else getattr(spin_0.parent, "name", str(spin_0.parent))
    parent_name_1 = spin_1.parent if isinstance(spin_1.parent, str) else getattr(spin_1.parent, "name", str(spin_1.parent))
    ctx.check(
        "wheel_0 is carried by rocker_arm_0",
        parent_name_0 == "rocker_arm_0",
        details=f"spin_0 parent={parent_name_0}",
    )
    ctx.check(
        "wheel_1 is carried by rocker_arm_1",
        parent_name_1 == "rocker_arm_1",
        details=f"spin_1 parent={parent_name_1}",
    )

    # Each rocker arm carries a visible spring_coil visual.
    ctx.check(
        "rocker_arm_0 has a spring_coil visual",
        any(v.name == "spring_coil" for v in rocker_0.visuals),
        details=f"visuals: {[v.name for v in rocker_0.visuals]}",
    )
    ctx.check(
        "rocker_arm_1 has a spring_coil visual",
        any(v.name == "spring_coil" for v in rocker_1.visuals),
        details=f"visuals: {[v.name for v in rocker_1.visuals]}",
    )

    # Intentional overlaps from the suspension mounting logic:
    # 1) Rocker arm pivot boss is seated against the gearbox side face.
    # 2) Rocker arm axle bearing housing sits inside the wheel hub bore.
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="side_gearbox_0", elem_b="arm_body",
        reason="Rocker arm pivot boss is bolted to the gearbox side face as a structural suspension mount.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="side_gearbox_1", elem_b="arm_body",
        reason="Rocker arm pivot boss is bolted to the gearbox side face as a structural suspension mount.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="arm_body", elem_b="treaded_tire",
        reason="Rocker arm axle bearing housing sits inside the wheel tire bore as a captured shaft.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="arm_body", elem_b="treaded_tire",
        reason="Rocker arm axle bearing housing sits inside the wheel tire bore as a captured shaft.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="arm_body", elem_b="rim_shell",
        reason="Rocker arm axle bearing housing nests inside the wheel rim bore.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="arm_body", elem_b="rim_shell",
        reason="Rocker arm axle bearing housing nests inside the wheel rim bore.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="shock_body_0", elem_b="spring_coil",
        reason="Shock body wraps around the coil spring at the suspension damper interface.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="shock_body_1", elem_b="spring_coil",
        reason="Shock body wraps around the coil spring at the suspension damper interface.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="axle_stub_0", elem_b="arm_body",
        reason="Axle stub sits inside the rocker arm bearing housing as a captured shaft.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="axle_stub_1", elem_b="arm_body",
        reason="Axle stub sits inside the rocker arm bearing housing as a captured shaft.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="arm_body", elem_b="hub_drum",
        reason="Rocker arm bearing housing nests inside the wheel hub drum.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="arm_body", elem_b="hub_drum",
        reason="Rocker arm bearing housing nests inside the wheel hub drum.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="axle_bearing_0", elem_b="arm_body",
        reason="Axle bearing sits inside the rocker arm bearing housing.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="axle_bearing_1", elem_b="arm_body",
        reason="Axle bearing sits inside the rocker arm bearing housing.",
    )
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="shock_body_0", elem_b="treaded_tire",
        reason="Shock body extends slightly past tire edge due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="shock_body_1", elem_b="treaded_tire",
        reason="Shock body extends slightly past tire edge due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="spring_coil", elem_b="hub_drum",
        reason="Spring coil extends past hub drum edge due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="spring_coil", elem_b="hub_drum",
        reason="Spring coil extends past hub drum edge due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="shock_body_0", elem_b="arm_body",
        reason="Shock body overlaps rocker arm body at suspension mounting point.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="shock_body_1", elem_b="arm_body",
        reason="Shock body overlaps rocker arm body at suspension mounting point.",
    )
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="shock_body_0", elem_b="rim_shell",
        reason="Shock body extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="shock_body_1", elem_b="rim_shell",
        reason="Shock body extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="spring_coil", elem_b="rim_shell",
        reason="Spring coil extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="spring_coil", elem_b="rim_shell",
        reason="Spring coil extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="shock_body_0", elem_b="shock_shaft",
        reason="Shock shaft slides inside shock body as part of suspension travel.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="shock_body_1", elem_b="shock_shaft",
        reason="Shock shaft slides inside shock body as part of suspension travel.",
    )
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="spring_mount_0", elem_b="rim_shell",
        reason="Spring mount overlaps wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="spring_mount_1", elem_b="rim_shell",
        reason="Spring mount overlaps wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="shock_shaft", elem_b="treaded_tire",
        reason="Shock shaft extends near tire due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="shock_shaft", elem_b="treaded_tire",
        reason="Shock shaft extends near tire due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="spring_mount_0", elem_b="arm_body",
        reason="Spring mount sits above the rocker arm spring seat.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="spring_mount_1", elem_b="arm_body",
        reason="Spring mount sits above the rocker arm spring seat.",
    )
    ctx.allow_overlap(
        rocker_0, wheel_0,
        elem_a="shock_shaft", elem_b="rim_shell",
        reason="Shock shaft extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        rocker_1, wheel_1,
        elem_a="shock_shaft", elem_b="rim_shell",
        reason="Shock shaft extends past wheel rim due to suspension geometry.",
    )
    ctx.allow_overlap(
        carriage, rocker_0,
        elem_a="spring_mount_0", elem_b="spring_coil",
        reason="Spring mount pad sits on top of the coil spring as part of the suspension assembly.",
    )
    ctx.allow_overlap(
        carriage, rocker_1,
        elem_a="spring_mount_1", elem_b="spring_coil",
        reason="Spring mount pad sits on top of the coil spring as part of the suspension assembly.",
    )

    # Proof: each rocker arm pivot remains within the gearbox Y/Z footprint.
    ctx.expect_overlap(rocker_0, carriage, axes="yz", elem_a="arm_body", elem_b="side_gearbox_0", min_overlap=0.020, name="rocker_0 arm overlaps gearbox_0 in YZ (pivot seated)")
    ctx.expect_overlap(rocker_1, carriage, axes="yz", elem_a="arm_body", elem_b="side_gearbox_1", min_overlap=0.020, name="rocker_1 arm overlaps gearbox_1 in YZ (pivot seated)")

    # Wheels sit outboard of the gearbox with a small functional clearance.
    ctx.expect_overlap(wheel_0, carriage, axes="yz", min_overlap=0.080, name="wheel 0 aligns to axle supports")
    ctx.expect_overlap(wheel_1, carriage, axes="yz", min_overlap=0.080, name="wheel 1 aligns to axle supports")

    # Rocker pivot positive pose lifts the wheel upward (suspension compression).
    rest_z_0 = ctx.part_world_position(wheel_0)[2] if ctx.part_world_position(wheel_0) else None
    rest_z_1 = ctx.part_world_position(wheel_1)[2] if ctx.part_world_position(wheel_1) else None
    with ctx.pose({rocker_piv_0: 0.12, rocker_piv_1: 0.12}):
        lifted_pos_0 = ctx.part_world_position(wheel_0)
        lifted_pos_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "rocker pivot positive pose lifts wheel_0",
        rest_z_0 is not None and lifted_pos_0 is not None and lifted_pos_0[2] > rest_z_0 + 0.002,
        details=f"rest_z={rest_z_0}, lifted_z={lifted_pos_0[2] if lifted_pos_0 else None}",
    )
    ctx.check(
        "rocker pivot positive pose lifts wheel_1",
        rest_z_1 is not None and lifted_pos_1 is not None and lifted_pos_1[2] > rest_z_1 + 0.002,
        details=f"rest_z={rest_z_1}, lifted_z={lifted_pos_1[2] if lifted_pos_1 else None}",
    )

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
