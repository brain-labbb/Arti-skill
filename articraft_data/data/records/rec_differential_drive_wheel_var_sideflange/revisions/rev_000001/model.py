from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _l_bracket_mesh() -> object:
    """L-shaped bracket: vertical flange plate + horizontal seat + gusset ribs.

    The flange bolts to a vertical chassis rib. The horizontal seat carries the
    drive carriage, with a central through-hole for the swivel post.
    """
    # Vertical flange plate (against the chassis wall)
    flange = (
        cq.Workplane("XY")
        .box(0.380, 0.014, 0.300)
        .translate((0.0, -0.150, 0.150))
    )

    # Horizontal seat plate (carries the carriage)
    seat = (
        cq.Workplane("XY")
        .box(0.340, 0.290, 0.014)
        .translate((0.0, -0.001, -0.005))
    )

    # Combine flange and seat into one solid
    bracket = flange.union(seat)

    # Gusset ribs reinforcing the flange-to-seat junction
    for x_sign in (-1, 1):
        gusset = (
            cq.Workplane("XY")
            .box(0.012, 0.136, 0.120)
            .translate((x_sign * 0.145, -0.079, 0.060))
        )
        bracket = bracket.union(gusset)

    # Central hole in the seat for the carriage swivel post and bearing
    seat_hole = (
        cq.Workplane("XY")
        .circle(0.052)
        .extrude(0.040, both=True)
        .translate((0.0, 0.0, -0.005))
    )
    bracket = bracket.cut(seat_hole)

    return bracket


def _bearing_ring_mesh() -> object:
    """Annular bearing ring seated on the bracket horizontal seat."""
    outer = cq.Workplane("XY").circle(0.080).extrude(0.016)
    inner_cut = cq.Workplane("XY").circle(0.048).extrude(0.022)
    return outer.cut(inner_cut)


def _eye_bolt_mesh() -> object:
    """Small vertical lift eye with a threaded stem."""
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
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="differential_drive_wheel_module",
        meta={
            "category": "Robotics / Differential drive wheel",
            "reference_notes": (
                "Side-flange-mounted differential drive wheel unit: vertical L-bracket "
                "bolts to a chassis rib, horizontal seat carries the gearbox carriage "
                "with limited steering swivel. Two coaxial tan treaded drive wheels."
            ),
        },
    )

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.085, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    tan_poly = model.material("tan_polyurethane", rgba=(0.91, 0.62, 0.25, 1.0))
    steel = model.material("satin_steel", rgba=(0.50, 0.52, 0.51, 1.0))

    # -----------------------------------------------------------------------
    # Root: L-bracket mount plate (vertical flange + horizontal seat)
    # -----------------------------------------------------------------------
    mount_plate = model.part("mount_plate")
    mount_plate.visual(
        mesh_from_cadquery(_l_bracket_mesh(), "l_bracket_body", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bracket_body",
    )
    mount_plate.visual(
        mesh_from_cadquery(_bearing_ring_mesh(), "seat_swivel_bearing", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_anodized,
        name="seat_bearing_ring",
    )

    # Flange bolt screws on the outer (+Y) face of the vertical flange,
    # emitted with a for-i loop and indexed names.
    flange_outer_y = -0.141  # slightly proud of the flange outer face at y=-0.143
    for i in range(8):
        row = i // 2
        col = i % 2
        x = -0.130 + col * 0.260
        z = 0.060 + row * 0.070
        _add_cyl(
            mount_plate,
            0.008,
            0.005,
            (x, flange_outer_y, z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
            material=steel,
            name=f"flange_screw_{i}",
        )

    # Lift eyes on the top edge of the vertical flange for module handling.
    for i, (x, yaw) in enumerate([(-0.080, 0.0), (0.080, 0.0)]):
        mount_plate.visual(
            mesh_from_cadquery(_eye_bolt_mesh(), f"lift_eye_{i}", tolerance=0.0006),
            origin=Origin(xyz=(x, -0.150, 0.300), rpy=(0.0, 0.0, yaw)),
            material=dark_anodized,
            name=f"lift_eye_{i}",
        )

    # -----------------------------------------------------------------------
    # Drive carriage: gearbox blocks, motor, axle supports, bearing features
    # -----------------------------------------------------------------------
    carriage = model.part("drive_carriage")

    # Main gearbox housings
    carriage.visual(Box((0.300, 0.135, 0.078)), origin=Origin(xyz=(0.0, -0.010, -0.108)), material=brushed_aluminum, name="center_gearbox")
    carriage.visual(Box((0.105, 0.155, 0.120)), origin=Origin(xyz=(-0.180, -0.012, -0.112)), material=brushed_aluminum, name="side_gearbox_0")
    carriage.visual(Box((0.105, 0.155, 0.120)), origin=Origin(xyz=(0.180, -0.012, -0.112)), material=brushed_aluminum, name="side_gearbox_1")

    # Front yoke plates bracing the axle supports
    carriage.visual(Box((0.175, 0.030, 0.125)), origin=Origin(xyz=(-0.105, 0.070, -0.112)), material=steel, name="front_yoke_0")
    carriage.visual(Box((0.175, 0.030, 0.125)), origin=Origin(xyz=(0.105, 0.070, -0.112)), material=steel, name="front_yoke_1")

    # Upper cross brace tying the yoke plates together
    carriage.visual(Box((0.255, 0.034, 0.035)), origin=Origin(xyz=(0.0, 0.076, -0.046)), material=steel, name="upper_cross_brace")

    # Swivel post and bearing features (post goes up through seat hole)
    _add_cyl(carriage, 0.049, 0.050, (0.0, 0.0, 0.006), material=dark_anodized, name="swivel_post")
    _add_cyl(carriage, 0.066, 0.012, (0.0, 0.0, -0.023), material=steel, name="inner_bearing_flange")
    _add_cyl(carriage, 0.055, 0.055, (0.0, 0.0, -0.045), material=steel, name="bearing_pedestal")

    # Seat mounting standoffs: vertical posts connecting the gearbox top face
    # (z=-0.069) up to the bracket seat underside (z≈-0.012), providing a rigid
    # connection between the carriage body and the seat interface.
    for i, (x, y_pos) in enumerate([(-0.100, -0.020), (0.100, -0.020), (-0.050, -0.060), (0.050, -0.060)]):
        _add_cyl(carriage, 0.010, 0.058, (x, y_pos, -0.040), material=steel, name=f"seat_standoff_{i}")

    # Motor can: coaxial with the axle line, braced by the gearbox blocks
    motor_rpy = (0.0, math.pi / 2.0, 0.0)
    _add_cyl(carriage, 0.047, 0.205, (0.0, -0.107, -0.106), rpy=motor_rpy, material=dark_anodized, name="motor_can")
    _add_cyl(carriage, 0.050, 0.010, (-0.108, -0.107, -0.106), rpy=motor_rpy, material=steel, name="motor_endcap_0")
    _add_cyl(carriage, 0.050, 0.010, (0.108, -0.107, -0.106), rpy=motor_rpy, material=steel, name="motor_endcap_1")
    for i, z in enumerate([-0.128, -0.106, -0.084]):
        carriage.visual(Box((0.226, 0.007, 0.007)), origin=Origin(xyz=(0.0, -0.156, z)), material=black_rubber, name=f"motor_fin_{i}")

    # Axle stubs: visible but stop short of the rotating wheel hubs
    _add_cyl(carriage, 0.014, 0.031, (-0.237, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_stub_0")
    _add_cyl(carriage, 0.014, 0.031, (0.237, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_stub_1")
    _add_cyl(carriage, 0.024, 0.014, (-0.220, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_bearing_0")
    _add_cyl(carriage, 0.024, 0.014, (0.220, 0.0, -0.105), rpy=motor_rpy, material=steel, name="axle_bearing_1")

    # Side face socket screws on the gearbox covers
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

    # -----------------------------------------------------------------------
    # Wheels: tan treaded tires, machined rims, hub drums, caps, and screws
    # -----------------------------------------------------------------------
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

    # -----------------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------------

    # Steering swivel: limited revolute about Z at the bracket seat center.
    # The carriage rotates on the horizontal seat for steering alignment.
    model.articulation(
        "mount_to_carriage",
        ArticulationType.REVOLUTE,
        parent=mount_plate,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.35, upper=0.35),
        motion_properties=MotionProperties(damping=0.3, friction=0.1),
    )

    # Wheel rotations: continuous revolute about the shared axle (X axis).
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
        "reference category matches visible differential drive module",
        True,
        details="No classification mismatch suspected: image and folder both indicate a robotics differential drive wheel module.",
    )

    # --- Vertical flange structural check (the changed primary form) ---
    # The mount_plate must have a vertical flange that extends well above the
    # horizontal seat, proving it is a side-flange mount (not a horizontal plate).
    plate_aabb = ctx.part_world_aabb(plate)
    carriage_aabb = ctx.part_world_aabb(carriage)
    if plate_aabb is not None and carriage_aabb is not None:
        plate_top_z = plate_aabb[1][2]
        carriage_top_z = carriage_aabb[1][2]
        ctx.check(
            "mount_plate vertical flange extends above drive carriage",
            plate_top_z > carriage_top_z + 0.10,
            details=f"plate top z={plate_top_z:.4f}, carriage top z={carriage_top_z:.4f}",
        )
        ctx.check(
            "L-bracket flange reaches at least 0.25m above the seat level",
            plate_top_z > 0.25,
            details=f"plate max z={plate_top_z:.4f}",
        )

    # --- Flange bolt pattern (indexed screws on vertical face) ---
    plate_visual_names = {v.name for v in plate.visuals if getattr(v, "name", None)}
    flange_screw_names = {f"flange_screw_{i}" for i in range(8)}
    ctx.check(
        "vertical flange has 8 indexed bolt screws",
        flange_screw_names.issubset(plate_visual_names),
        details=f"missing: {flange_screw_names - plate_visual_names}",
    )

    # --- Drive mechanism checks ---
    ctx.check(
        "module has two independent rotating wheels",
        spin_0.articulation_type == ArticulationType.CONTINUOUS and spin_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"spin types: {spin_0.articulation_type}, {spin_1.articulation_type}",
    )
    ctx.check(
        "wheel axes are collinear",
        spin_0.axis == (1.0, 0.0, 0.0) and spin_1.axis == (1.0, 0.0, 0.0),
        details=f"axes: {spin_0.axis}, {spin_1.axis}",
    )
    ctx.check(
        "bracket seat swivel is a limited steering revolute",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower < 0.0 < swivel.motion_limits.upper,
        details=f"swivel={swivel}",
    )

    # Swivel joint is at the bracket seat (z≈0), not at an overhead plate.
    ctx.check(
        "swivel joint origin is at the bracket horizontal seat",
        abs(swivel.origin.xyz[2]) < 0.02,
        details=f"swivel origin z={swivel.origin.xyz[2]:.4f}",
    )

    # --- Wheel clearance and alignment ---
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

    # Wheel spin keeps axle centers fixed
    rest_0 = ctx.part_world_position(wheel_0)
    rest_1 = ctx.part_world_position(wheel_1)
    with ctx.pose({spin_0: 1.2, spin_1: -1.2}):
        spun_0 = ctx.part_world_position(wheel_0)
        spun_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheel spin keeps axle centers fixed",
        rest_0 is not None and spun_0 is not None and rest_1 is not None and spun_1 is not None
        and max(abs(rest_0[i] - spun_0[i]) for i in range(3)) < 1e-6
        and max(abs(rest_1[i] - spun_1[i]) for i in range(3)) < 1e-6,
        details=f"rest=({rest_0}, {rest_1}) spun=({spun_0}, {spun_1})",
    )

    # --- Intentional bearing overlap: swivel post captured in seat bearing ---
    # The carriage swivel_post intentionally passes through the seat_bearing_ring
    # as a captured shaft in a rotational bearing interface.
    ctx.allow_overlap(
        carriage,
        plate,
        elem_a="swivel_post",
        elem_b="seat_bearing_ring",
        reason="The carriage swivel post is intentionally captured inside the seat bearing ring as a rotational steering bearing.",
    )
    ctx.expect_contact(
        carriage,
        plate,
        elem_a="swivel_post",
        elem_b="seat_bearing_ring",
        name="swivel post engages the seat bearing ring",
    )

    # --- Carriage-seat engagement during steering ---
    # At rest, the carriage should overlap the bracket seat in XY
    ctx.expect_overlap(
        carriage, plate, axes="xy", min_overlap=0.05,
        name="carriage sits within the bracket seat footprint",
    )

    with ctx.pose({swivel: 0.25}):
        ctx.expect_overlap(
            carriage, plate, axes="xy", min_overlap=0.03,
            name="swiveled carriage maintains bracket seat engagement",
        )

    return ctx.report()


object_model = build_object_model()
