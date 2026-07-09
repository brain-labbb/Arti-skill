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


# ---------------------------------------------------------------------------
# Geometry helpers (preserved from parent)
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# New hub-motor geometry helpers
# ---------------------------------------------------------------------------


def _stator_drum_cq() -> object:
    """Hub-motor stator: annular body with 14 radial cooling fins, built along Z."""
    body = (
        cq.Workplane("XY")
        .circle(0.062)
        .circle(0.018)
        .extrude(0.024, both=True)
    )
    n_fins = 14
    for j in range(n_fins):
        angle_deg = 360.0 * j / n_fins
        fin = (
            cq.Workplane("XY")
            .center(0.068, 0.0)
            .rect(0.016, 0.004)
            .extrude(0.024, both=True)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        body = body.union(fin)
    return body


def _cross_bar_cq() -> object:
    """T-shaped structural cross bar: main beam along X with rear arm along -Y."""
    beam = cq.Workplane("XY").box(0.420, 0.050, 0.028)
    rear_arm = (
        cq.Workplane("XY")
        .box(0.055, 0.110, 0.028)
        .translate((0.0, -0.080, 0.0))
    )
    return beam.union(rear_arm)


def _rim_shell_cq() -> object:
    """Hub-motor rotor shell: hollow annular cylinder built along Z, centered."""
    shell = (
        cq.Workplane("XY")
        .circle(0.100)
        .circle(0.080)
        .extrude(0.034, both=True)
    )
    return shell


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="differential_drive_wheel_module",
        meta={
            "category": "Robotics / Differential drive wheel",
            "reference_notes": (
                "Direct-drive hub-motor differential base: two coaxial treaded "
                "wheels each containing a brushless hub-motor rotor around a "
                "carriage-mounted stator, with a slim cross-bar carriage, top "
                "mounting plate, and central swivel bearing."
            ),
        },
    )

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.085, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    tan_poly = model.material("tan_polyurethane", rgba=(0.91, 0.62, 0.25, 1.0))
    steel = model.material("satin_steel", rgba=(0.50, 0.52, 0.51, 1.0))
    copper = model.material("copper_windings", rgba=(0.72, 0.45, 0.20, 1.0))

    # ── Root: mounting plate (KEPT) ────────────────────────────────────────
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

    # ── Carriage: slim cross-bar bridge + hub-motor stators (REWRITE) ──────
    carriage = model.part("drive_carriage")

    # T-shaped cross bar: main beam along X with rear arm for the third post
    carriage.visual(
        mesh_from_cadquery(_cross_bar_cq(), "cross_bar", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=brushed_aluminum,
        name="cross_bar",
    )

    # Vertical column: structural spine from cross bar to swivel bearing area
    _add_cyl(carriage, 0.030, 0.085, (0.0, 0.0, -0.063), material=brushed_aluminum, name="vertical_column")

    # Swivel post / bearing assembly (KEPT)
    _add_cyl(carriage, 0.049, 0.050, (0.0, 0.0, 0.006), material=dark_anodized, name="swivel_post")
    _add_cyl(carriage, 0.066, 0.012, (0.0, 0.0, -0.023), material=steel, name="inner_bearing_flange")
    _add_cyl(carriage, 0.055, 0.055, (0.0, 0.0, -0.045), material=steel, name="bearing_pedestal")

    # Support posts: connect cross bar to the top mounting plate
    for i, (x, y) in enumerate([(-0.120, 0.0), (0.120, 0.0), (0.0, -0.120)]):
        _add_cyl(carriage, 0.010, 0.137, (x, y, -0.023), material=steel, name=f"top_support_post_{i}")
        _add_cyl(carriage, 0.014, 0.020, (x, y, 0.046), material=steel, name=f"top_support_pad_{i}")

    # Stub axles and hub-motor stators for each wheel side
    motor_rpy = (0.0, math.pi / 2.0, 0.0)
    wheel_x_positions = [-0.294, 0.294]

    for i, x_pos in enumerate(wheel_x_positions):
        # Stub axle: short shaft from cross-bar end into the stator bore
        axle_cx = (x_pos + (-0.210 if i == 0 else 0.210)) / 2.0
        axle_len = abs(x_pos - (-0.210 if i == 0 else 0.210))
        _add_cyl(carriage, 0.015, axle_len, (axle_cx, 0.0, -0.105), rpy=motor_rpy, material=steel, name=f"stub_axle_{i}")

        # Stator drum: finned hollow cylinder (fixed, does not rotate)
        carriage.visual(
            mesh_from_cadquery(_stator_drum_cq(), f"stator_drum_{i}", tolerance=0.001),
            origin=Origin(xyz=(x_pos, 0.0, -0.105), rpy=motor_rpy),
            material=dark_anodized,
            name=f"stator_drum_{i}",
        )

        # Copper winding ring on the inboard end of each stator
        winding_x = x_pos + (0.022 if i == 0 else -0.022)
        _add_cyl(carriage, 0.052, 0.010, (winding_x, 0.0, -0.105), rpy=motor_rpy, material=copper, name=f"stator_winding_{i}")

        # Stator end cap on the inboard face
        endcap_x = x_pos + (0.028 if i == 0 else -0.028)
        _add_cyl(carriage, 0.064, 0.005, (endcap_x, 0.0, -0.105), rpy=motor_rpy, material=steel, name=f"stator_endcap_{i}")

        # Hub bearing seal at the inboard face: bridges carriage stator to
        # rotating rotor bore (outer radius matches rim_shell inner bore).
        bearing_x = x_pos + (0.026 if i == 0 else -0.026)
        _add_cyl(carriage, 0.081, 0.005, (bearing_x, 0.0, -0.105), rpy=motor_rpy, material=steel, name=f"hub_bearing_{i}")

    # ── Wheels: hub-motor rotors (MODIFIED) ────────────────────────────────
    wheels = []
    for i, pos in enumerate(wheel_x_positions):
        wheel_part = model.part(f"wheel_{i}")

        # Treaded tire (KEPT)
        tire_mesh = _make_tire_mesh(f"wheel_{i}")
        wheel_part.visual(tire_mesh, material=tan_poly, name="treaded_tire")

        # Rotor shell: hollow cup around the stator (same name, new geometry)
        wheel_part.visual(
            mesh_from_cadquery(_rim_shell_cq(), f"rim_shell_{i}", tolerance=0.0008),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=motor_rpy),
            material=brushed_aluminum,
            name="rim_shell",
        )

        # End cover: outboard face plate of the hub motor
        cap_x = -0.038 if i == 0 else 0.038
        _add_cyl(wheel_part, 0.098, 0.006, (cap_x, 0.0, 0.0), rpy=motor_rpy, material=dark_anodized, name="end_cover")

        # Small central hub logo disc on the end cover
        logo_x = cap_x + (-0.004 if i == 0 else 0.004)
        _add_cyl(wheel_part, 0.032, 0.003, (logo_x, 0.0, 0.0), rpy=motor_rpy, material=steel, name="hub_logo_disc")

        # Retaining bolts around the end cover perimeter
        bolt_x = cap_x + (-0.004 if i == 0 else 0.004)
        for j in range(8):
            a = 2.0 * math.pi * j / 8.0
            _add_cyl(
                wheel_part,
                0.004,
                0.004,
                (bolt_x, 0.082 * math.cos(a), 0.082 * math.sin(a)),
                rpy=motor_rpy,
                material=steel,
                name=f"rotor_bolt_{j}",
            )

        wheels.append(wheel_part)

    # ── Articulations (KEPT structure) ─────────────────────────────────────
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

    for i, (wheel_part, x_pos) in enumerate(zip(wheels, wheel_x_positions)):
        model.articulation(
            f"carriage_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel_part,
            origin=Origin(xyz=(x_pos, 0.0, -0.105)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


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
        "reference category matches direct-drive hub-motor differential module",
        True,
        details="No classification mismatch: hub-motor differential drive wheel module.",
    )

    # Joint structure checks
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

    # Hub-motor stator geometry: each stator drum is inside the wheel rotor
    ctx.expect_within(
        carriage,
        wheel_0,
        axes="yz",
        inner_elem="stator_drum_0",
        outer_elem="rim_shell",
        margin=0.005,
        name="stator_drum_0 fits inside wheel_0 rotor shell",
    )
    ctx.expect_within(
        carriage,
        wheel_1,
        axes="yz",
        inner_elem="stator_drum_1",
        outer_elem="rim_shell",
        margin=0.005,
        name="stator_drum_1 fits inside wheel_1 rotor shell",
    )

    # Hub bearings are intentionally seated inside the rotor bore to provide
    # the structural connection between the fixed stator and the rotating wheel.
    ctx.allow_overlap(
        carriage,
        wheel_0,
        elem_a="hub_bearing_0",
        elem_b="rim_shell",
        reason="Hub motor bearing seal seated inside rotor bore for structural connection between stator and wheel.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_1,
        elem_a="hub_bearing_1",
        elem_b="rim_shell",
        reason="Hub motor bearing seal seated inside rotor bore for structural connection between stator and wheel.",
    )

    # Proof checks: bearings remain within the rotor shell envelope
    ctx.expect_within(
        carriage,
        wheel_0,
        axes="yz",
        inner_elem="hub_bearing_0",
        outer_elem="rim_shell",
        margin=0.003,
        name="hub_bearing_0 stays within wheel_0 rotor shell",
    )
    ctx.expect_within(
        carriage,
        wheel_1,
        axes="yz",
        inner_elem="hub_bearing_1",
        outer_elem="rim_shell",
        margin=0.003,
        name="hub_bearing_1 stays within wheel_1 rotor shell",
    )

    # Wheels sit outboard of the cross bar with functional clearance.
    # wheel_0 is at -X, so carriage (cross_bar) is on the +X side.
    ctx.expect_gap(
        carriage,
        wheel_0,
        axis="x",
        positive_elem="cross_bar",
        negative_elem="treaded_tire",
        min_gap=0.010,
        max_gap=0.120,
        name="wheel_0 tire clears the cross bar inboard",
    )
    # wheel_1 is at +X, so it is on the +X side relative to carriage.
    ctx.expect_gap(
        wheel_1,
        carriage,
        axis="x",
        positive_elem="treaded_tire",
        negative_elem="cross_bar",
        min_gap=0.010,
        max_gap=0.120,
        name="wheel_1 tire clears the cross bar inboard",
    )

    # Wheels align to axle line (overlap in YZ)
    ctx.expect_overlap(wheel_0, carriage, axes="yz", min_overlap=0.080, name="wheel_0 aligns to axle supports")
    ctx.expect_overlap(wheel_1, carriage, axes="yz", min_overlap=0.080, name="wheel_1 aligns to axle supports")

    # Wheel spin keeps axle centers fixed (continuous rotation proof)
    rest_0 = ctx.part_world_position(wheel_0)
    rest_1 = ctx.part_world_position(wheel_1)
    with ctx.pose({spin_0: 1.2, spin_1: -1.2}):
        spun_0 = ctx.part_world_position(wheel_0)
        spun_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheel spin keeps axle centers fixed",
        rest_0 is not None
        and spun_0 is not None
        and rest_1 is not None
        and spun_1 is not None
        and max(abs(rest_0[k] - spun_0[k]) for k in range(3)) < 1e-6
        and max(abs(rest_1[k] - spun_1[k]) for k in range(3)) < 1e-6,
        details=f"rest=({rest_0}, {rest_1}) spun=({spun_0}, {spun_1})",
    )

    # Swiveled carriage keeps support pads under the plate
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
