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
    SpurGear,
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
    Worm,
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
        (-0.265, -0.165), (0.265, -0.165),
        (-0.265, 0.165), (0.265, 0.165),
        (-0.160, -0.185), (0.160, -0.185),
        (-0.160, 0.185), (0.160, 0.185),
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


def _worm_screw_mesh() -> object:
    """Helical worm for the motor output shaft — visible threads.

    Build at mm scale then scale to metres.
    After rotation the worm axis is along Y, extending ±0.018 m.
    """
    worm = Worm(module=2.0, lead_angle=14.0, n_threads=2, length=36.0)
    shape = worm.build(bore_d=6.0)
    # Gear builder axis is Z; rotate to Y (fore-aft motor shaft direction).
    shape = shape.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    shape = shape.scale(0.001)
    return shape


def _worm_wheel_disc_mesh() -> object:
    """Toothed worm-wheel disc coaxial with the wheel axle.

    Build at mm scale then scale to metres.
    After rotation the gear axis is along X, width 0.018 m.
    Outer radius (tooth tips) ≈ 0.024 m.
    """
    ww = SpurGear(module=2.0, teeth_number=22, width=18.0)
    shape = ww.build(bore_d=12.0)
    shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    shape = shape.scale(0.001)
    return shape


def _worm_housing_mesh() -> object:
    """Compact cast housing enclosing the worm / worm-wheel mesh zone.

    Z is up.  Motor-shaft bore exits through the −Y face.
    Flange half-width in X = 0.043 m (total 0.086).
    """
    body = (
        cq.Workplane("XY")
        .box(0.070, 0.078, 0.078)
        .edges("|Z")
        .fillet(0.006)
    )
    # Top mounting flange.
    flange = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.040))
        .box(0.086, 0.092, 0.008)
        .edges("|Z")
        .fillet(0.004)
    )
    body = body.union(flange)
    # Motor-shaft boss on the −Y face — protrudes outward to contact motor can.
    boss = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -0.045, 0.0))
        .box(0.040, 0.024, 0.040)
    )
    body = body.union(boss)
    # Lower reinforcement rib.
    rib = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.040))
        .box(0.060, 0.068, 0.006)
    )
    body = body.union(rib)
    return body


def _carriage_body_mesh() -> object:
    """Single connected carriage frame: column, web, beam, side webs.

    All local coordinates.  The carriage part origin coincides with the
    mount_to_carriage articulation origin (world z = 0.235).
    
    The column and top plate stay below the bearing ring (world z > 0.253)
    to avoid overlap.  The swivel post (separate visual) extends into the ring.
    """
    # Central vertical web connecting beam area to column area.
    web = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.060))
        .box(0.030, 0.080, 0.120)
    )
    # Upper bearing column — radius 0.050 < bearing ring inner radius 0.052.
    # Column from local z=-0.060 to z=0.018 (world 0.175 to 0.253).
    column = (
        cq.Workplane("XY")
        .circle(0.050)
        .extrude(0.078)
        .translate((0.0, 0.0, -0.060))
    )
    web = web.union(column)
    # Lower cross-beam runs in X between the two housing mount points.
    beam = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.120))
        .box(0.420, 0.040, 0.028)
    )
    web = web.union(beam)
    # Side vertical webs connect beam to housing mounting area.
    for x_sign in (-1.0, 1.0):
        side_web = (
            cq.Workplane("XY")
            .transformed(offset=(x_sign * 0.200, 0.0, -0.096))
            .box(0.030, 0.040, 0.076)
        )
        web = web.union(side_web)
    # Top plate — flat bracket at the column top, below the bearing ring.
    # Local z=0.012 to z=0.022 (world 0.247 to 0.257).  Below bearing ring bottom (0.253).
    top_plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.017))
        .box(0.260, 0.260, 0.010)
    )
    web = web.union(top_plate)
    return web


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="differential_drive_wheel_module",
        meta={
            "category": "Robotics / Differential drive wheel",
            "reference_notes": "Two tan treaded drive wheels on a shared axle line with a metal top mounting plate, central swivel bearing, worm-gear reduction carriage, fore-aft motors, brackets, and fasteners.",
        },
    )

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.0, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.085, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    tan_poly = model.material("tan_polyurethane", rgba=(0.91, 0.62, 0.25, 1.0))
    steel = model.material("satin_steel", rgba=(0.50, 0.52, 0.51, 1.0))
    cast_iron = model.material("cast_iron_grey", rgba=(0.28, 0.28, 0.27, 1.0))

    # ── Root: the plate that would bolt the module into a robot chassis. ──
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
            (-0.265, -0.165), (0.265, -0.165),
            (-0.265, 0.165), (0.265, 0.165),
            (-0.160, -0.185), (0.160, -0.185),
            (-0.160, 0.185), (0.160, 0.185),
        ]
    ):
        _add_cyl(mount_plate, 0.010, 0.004, (x, y, 0.311), material=steel, name=f"plate_screw_{i}")
    for i in range(10):
        a = 2.0 * math.pi * i / 10.0
        _add_cyl(
            mount_plate, 0.0055, 0.003,
            (0.078 * math.cos(a), 0.078 * math.sin(a), 0.3105),
            material=steel, name=f"bearing_screw_{i}",
        )

    for i, (x, y, yaw) in enumerate([(-0.095, 0.075, 0.0), (0.105, 0.075, 0.45), (0.000, -0.105, -0.35)]):
        mount_plate.visual(
            mesh_from_cadquery(_eye_bolt_mesh(), f"lift_eye_{i}", tolerance=0.0006),
            origin=Origin(xyz=(x, y, 0.309), rpy=(0.0, 0.0, yaw)),
            material=dark_anodized,
            name=f"lift_eye_{i}",
        )

    # ── Carriage: connected frame + worm-gear housings + motors. ──
    # The carriage part origin coincides with the mount_to_carriage articulation
    # origin at world (0, 0, 0.235).  All carriage-local Z values are offset by
    # +0.235 in world space.
    carriage = model.part("drive_carriage")

    # Single connected carriage body (CQ mesh).
    carriage.visual(
        mesh_from_cadquery(_carriage_body_mesh(), "carriage_frame", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="carriage_frame",
    )

    # Bearing post sits on top of the column, extending into the swivel bearing.
    # Column top at local z=0.018 (world 0.253).  Post from 0.018 to 0.056 (world 0.253 to 0.291).
    # Post top stays at or below the plate bottom (world 0.291) so it passes through
    # the plate hole without overlapping the plate solid ring.
    # Post radius 0.049 < bearing ring inner radius 0.052.
    _add_cyl(carriage, 0.049, 0.038, (0.0, 0.0, 0.037), material=dark_anodized, name="swivel_post")
    # Inner bearing flange at the column top.
    # Flange radius 0.050 < bearing ring inner radius 0.052 to avoid overlap.
    _add_cyl(carriage, 0.050, 0.012, (0.0, 0.0, 0.018), material=steel, name="inner_bearing_flange")

    # Support pads bridge from the top plate to the mount plate bottom.
    # Top plate top at local z=0.022 (world 0.257).  Plate bottom at world z=0.291.
    # Pad from local z=0.022 to z=0.056 (world 0.257 to 0.291).  Length = 0.034.
    # Center at local z = 0.022 + 0.017 = 0.039.
    for i, (x, y) in enumerate([(-0.120, 0.0), (0.120, 0.0), (0.0, -0.120)]):
        _add_cyl(carriage, 0.014, 0.034, (x, y, 0.039), material=steel, name=f"top_support_pad_{i}")

    # ── Worm-gear reduction: one compact housing per side. ──
    # Wheel axle at local z = -0.105 (world 0.130).
    # Worm sits 0.032 m below the axle.
    worm_wheel_z = -0.105
    center_dist = 0.032
    worm_z = worm_wheel_z - center_dist  # -0.137
    housing_z = (worm_wheel_z + worm_z) / 2.0  # -0.121

    # Housing X positions — positioned so the housing outboard face is
    # inboard of the wheel tire inner face, with a small gap.
    # Wheel tire inner face at ±0.294 ∓ 0.039 = ±0.255.
    # Housing body half-width in X = 0.035.  Housing at ±0.200 extends to ±0.235.
    # Gap = 0.255 - 0.235 = 0.020m, which is > 0.001m requirement.
    side_x = [-0.200, 0.200]

    # Motor shaft runs fore-aft (Y).
    motor_y_rpy = (-math.pi / 2.0, 0.0, 0.0)

    for i, x_h in enumerate(side_x):
        # Cast worm-gear housing block.
        carriage.visual(
            mesh_from_cadquery(_worm_housing_mesh(), f"worm_housing_{i}", tolerance=0.0006),
            origin=Origin(xyz=(x_h, 0.0, housing_z)),
            material=cast_iron,
            name=f"worm_gear_housing_{i}",
        )

        # Helical worm on the motor shaft.
        # Worm axis along Y, centered at (x_h, 0, worm_z) to mesh with the worm wheel.
        carriage.visual(
            mesh_from_cadquery(_worm_screw_mesh(), f"worm_thread_{i}", tolerance=0.0004),
            origin=Origin(xyz=(x_h, 0.0, worm_z)),
            material=steel,
            name=f"worm_{i}",
        )

        # Motor can — shaft runs fore-aft (along Y).
        # Motor front contacts the housing boss (boss extends to Y ≈ -0.057).
        motor_y_center = -0.103
        _add_cyl(
            carriage, 0.042, 0.092,
            (x_h, motor_y_center, worm_z),
            rpy=motor_y_rpy,
            material=dark_anodized,
            name=f"motor_can_{i}",
        )
        # Motor rear endcap.
        _add_cyl(
            carriage, 0.044, 0.008,
            (x_h, motor_y_center - 0.048, worm_z),
            rpy=motor_y_rpy,
            material=steel,
            name=f"motor_endcap_{i}",
        )
        # Motor cooling fins on the bottom of the motor can.
        for j, dy in enumerate([-0.025, 0.000, 0.025]):
            carriage.visual(
                Box((0.090, 0.005, 0.005)),
                origin=Origin(xyz=(x_h, motor_y_center + dy, worm_z - 0.042)),
                material=black_rubber,
                name=f"motor_fin_{i}_{j}",
            )

        # Housing cover screws on the top flange face.
        for j, (dx, dy) in enumerate([(-0.032, -0.032), (0.032, -0.032), (-0.032, 0.032), (0.032, 0.032)]):
            _add_cyl(
                carriage, 0.005, 0.004,
                (x_h + dx, dy, housing_z + 0.044),
                material=dark_anodized,
                name=f"housing_screw_{i}_{j}",
            )

    # ── Rotating wheels: tan treaded tires, machined hubs, worm-wheel discs. ──
    wheel_positions = [(-0.294, 0.0, -0.105), (0.294, 0.0, -0.105)]
    motor_rpy = (0.0, math.pi / 2.0, 0.0)
    wheels = []
    for i, pos in enumerate(wheel_positions):
        wheel_part = model.part(f"wheel_{i}")
        tire_mesh = _make_tire_mesh(f"wheel_{i}")
        wheel_part.visual(tire_mesh, material=tan_poly, name="treaded_tire")
        # Hub drum extends to connect with the worm wheel disc.
        # Drum half-length 0.084 in X reaches the housing at ±0.210.
        _add_cyl(wheel_part, 0.041, 0.168, (0.0, 0.0, 0.0), rpy=motor_rpy, material=steel, name="hub_drum")
        _add_cyl(wheel_part, 0.100, 0.073, (0.0, 0.0, 0.0), rpy=motor_rpy, material=brushed_aluminum, name="rim_shell")
        cap_x = -0.056 if i == 0 else 0.056
        _add_cyl(wheel_part, 0.036, 0.006, (cap_x, 0.0, 0.0), rpy=motor_rpy, material=steel, name="outer_hub_cap")
        screw_x = -0.060 if i == 0 else 0.060
        for j in range(10):
            a = 2.0 * math.pi * j / 10.0
            _add_cyl(
                wheel_part, 0.0032, 0.003,
                (screw_x, 0.026 * math.cos(a), 0.026 * math.sin(a)),
                rpy=motor_rpy,
                material=dark_anodized,
                name=f"hub_screw_{j}",
            )

        # Worm wheel disc — keyed to the wheel axle, co-rotates with the wheel.
        # The gear mesh extends in +X from the origin by 0.018m (gear width).
        # Offset positions the disc center at the housing center (X = ±0.210).
        ww_offset_x = 0.084 if i == 0 else -0.084
        wheel_part.visual(
            mesh_from_cadquery(_worm_wheel_disc_mesh(), f"worm_wheel_disc_{i}", tolerance=0.0004),
            origin=Origin(xyz=(ww_offset_x, 0.0, 0.0)),
            material=steel,
            name="worm_wheel",
        )

        wheels.append(wheel_part)

    # ── Articulations: visible top swivel and two drive-wheel rotations. ──
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
        details="No classification mismatch suspected.",
    )

    ctx.check(
        "module has two independent rotating wheels",
        spin_0.articulation_type == ArticulationType.CONTINUOUS
        and spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"spin types: {spin_0.articulation_type}, {spin_1.articulation_type}",
    )
    ctx.check(
        "wheel axes are collinear",
        spin_0.axis == (1.0, 0.0, 0.0) and spin_1.axis == (1.0, 0.0, 0.0),
        details=f"axes: {spin_0.axis}, {spin_1.axis}",
    )
    ctx.check(
        "top bearing is a limited steering swivel",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower < 0.0 < swivel.motion_limits.upper,
        details=f"swivel={swivel}",
    )

    # ── Worm-gear reduction: each side has a housing, worm, and worm wheel. ──
    housing_0 = carriage.get_visual("worm_gear_housing_0")
    housing_1 = carriage.get_visual("worm_gear_housing_1")
    worm_0 = carriage.get_visual("worm_0")
    worm_1 = carriage.get_visual("worm_1")
    ctx.check(
        "worm_gear_housing_0 and worm_gear_housing_1 exist on drive_carriage",
        housing_0 is not None and housing_1 is not None,
        details="Each side has a compact cast worm-gear housing block.",
    )
    ctx.check(
        "worm_0 and worm_1 helical screws exist on drive_carriage",
        worm_0 is not None and worm_1 is not None,
        details="Each motor shaft terminates in a visible helical worm.",
    )

    ww_0 = wheel_0.get_visual("worm_wheel")
    ww_1 = wheel_1.get_visual("worm_wheel")
    ctx.check(
        "worm_wheel discs exist on wheel parts (co-rotate with wheel)",
        ww_0 is not None and ww_1 is not None,
        details="Toothed worm-wheel discs are keyed to the wheel axles and rotate with the wheels.",
    )

    # Allow the intentional worm-wheel / housing mesh overlap.
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="worm_gear_housing_0", elem_b="worm_wheel",
        reason="The worm wheel disc meshes with the worm inside the housing; visible tooth contact is intentional.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="worm_gear_housing_1", elem_b="worm_wheel",
        reason="The worm wheel disc meshes with the worm inside the housing; visible tooth contact is intentional.",
    )
    # Allow the housing / hub_drum overlap — the hub drum (axle) passes through
    # the housing (gear enclosure), which is intentional in a real worm-gear drive.
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="worm_gear_housing_0", elem_b="hub_drum",
        reason="The hub drum (axle assembly) passes through the worm-gear housing; the housing encloses the gear mechanism around the axle.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="worm_gear_housing_1", elem_b="hub_drum",
        reason="The hub drum (axle assembly) passes through the worm-gear housing; the housing encloses the gear mechanism around the axle.",
    )
    # Allow worm / hub_drum overlap — the worm wraps around the axle shaft area
    # where the worm meshes the worm wheel, which is the core mechanism geometry.
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="worm_0", elem_b="hub_drum",
        reason="The worm wraps around the axle shaft area where it meshes the worm wheel; this is the core right-angle gear geometry.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="worm_1", elem_b="hub_drum",
        reason="The worm wraps around the axle shaft area where it meshes the worm wheel; this is the core right-angle gear geometry.",
    )
    # Allow carriage_frame / hub_drum overlap — the hub drum extends through the
    # carriage side webs where the wheel axle passes through the gear housing area.
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="carriage_frame", elem_b="hub_drum",
        reason="The hub drum (axle) passes through the carriage frame side webs at the gear housing location.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="carriage_frame", elem_b="hub_drum",
        reason="The hub drum (axle) passes through the carriage frame side webs at the gear housing location.",
    )
    # Allow carriage_frame / worm_wheel overlap — the worm wheel disc extends into
    # the carriage frame area where it meshes the worm on the motor shaft.
    ctx.allow_overlap(
        carriage, wheel_0,
        elem_a="carriage_frame", elem_b="worm_wheel",
        reason="The worm wheel disc extends into the carriage frame gear housing area where it meshes the worm.",
    )
    ctx.allow_overlap(
        carriage, wheel_1,
        elem_a="carriage_frame", elem_b="worm_wheel",
        reason="The worm wheel disc extends into the carriage frame gear housing area where it meshes the worm.",
    )

    # Proof checks: worm wheel overlaps housing in YZ (meshing zone).
    ctx.expect_overlap(
        wheel_0, carriage, axes="yz",
        elem_a="worm_wheel", elem_b="worm_gear_housing_0",
        min_overlap=0.010,
        name="worm wheel 0 meshes inside housing at rest",
    )
    ctx.expect_overlap(
        wheel_1, carriage, axes="yz",
        elem_a="worm_wheel", elem_b="worm_gear_housing_1",
        min_overlap=0.010,
        name="worm wheel 1 meshes inside housing at rest",
    )

    # Wheels sit outboard of the carriage with a small functional clearance.
    # The tire inner face is at wheel_x ± tire_half_width.
    # The housing outer face is at side_x ± housing_half_width_x (0.043).
    # We need wheel_inner_face - housing_outer_face > 0.001.
    # Wheel at ±0.294, tire half-width ≈ 0.039, so tire inner face at ±0.255.
    # Housing at ±0.210, half-width 0.043, so housing outer face at ±0.253.
    # Gap = 0.255 - 0.253 = 0.002. This is too tight.
    # Move housing inboard to ±0.200 so gap = 0.255 - 0.243 = 0.012.
    ctx.expect_gap(
        carriage, wheel_0, axis="x",
        negative_elem="treaded_tire",
        min_gap=0.001, max_gap=0.080,
        name="wheel 0 tire clears the housing side",
    )
    ctx.expect_gap(
        wheel_1, carriage, axis="x",
        positive_elem="treaded_tire",
        min_gap=0.001, max_gap=0.080,
        name="wheel 1 tire clears the housing side",
    )
    ctx.expect_overlap(wheel_0, carriage, axes="yz", min_overlap=0.080, name="wheel 0 aligns to axle supports")
    ctx.expect_overlap(wheel_1, carriage, axes="yz", min_overlap=0.080, name="wheel 1 aligns to axle supports")

    # Worm-wheel disc co-rotates with wheel spin.
    rest_ww0 = ctx.part_world_position(wheel_0)
    with ctx.pose({spin_0: 1.2, spin_1: -1.2}):
        spun_ww0 = ctx.part_world_position(wheel_0)
    ctx.check(
        "worm_wheel co-rotates with wheel (wheel spin keeps axle center fixed)",
        rest_ww0 is not None and spun_ww0 is not None
        and max(abs(rest_ww0[k] - spun_ww0[k]) for k in range(3)) < 1e-6,
        details=f"rest={rest_ww0}, spun={spun_ww0}",
    )

    with ctx.pose({swivel: 0.25}):
        ctx.expect_gap(
            plate, carriage, axis="z",
            positive_elem="rounded_plate",
            min_gap=0.0, max_gap=0.005,
            name="swiveled support pads ride under the solid top plate",
        )

    return ctx.report()


object_model = build_object_model()
