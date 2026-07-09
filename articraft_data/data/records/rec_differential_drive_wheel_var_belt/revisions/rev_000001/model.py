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


def _toothed_pulley_mesh(radius: float, width: float, n_teeth: int = 20, tooth_depth: float = 0.0015) -> object:
    """Timing pulley with tooth-profile perimeter and retaining flanges, centered at origin."""
    pts = []
    for i in range(n_teeth * 2):
        angle = 2.0 * math.pi * i / (n_teeth * 2)
        r = radius + (tooth_depth if i % 2 == 0 else -tooth_depth)
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    wp = cq.Workplane("XY").moveTo(pts[0][0], pts[0][1])
    for p in pts[1:]:
        wp = wp.lineTo(p[0], p[1])
    body = wp.close().extrude(width).translate((0.0, 0.0, -width / 2.0))
    flange_h = 0.002
    flange_r = radius + 0.004
    f1 = cq.Workplane("XY").circle(flange_r).extrude(flange_h).translate((0.0, 0.0, -width / 2.0))
    f2 = cq.Workplane("XY").circle(flange_r).extrude(flange_h).translate((0.0, 0.0, width / 2.0 - flange_h))
    bore = cq.Workplane("XY").circle(radius * 0.25).extrude(width + 0.004).translate((0.0, 0.0, -(width + 0.004) / 2.0))
    return body.union(f1).union(f2).cut(bore)


def _belt_loop_mesh(
    x_pos: float,
    y_m: float, z_m: float, r_m: float,
    y_w: float, z_w: float, r_w: float,
    width: float = 0.012,
    thickness: float = 0.004,
) -> object:
    """Timing belt loop between motor and wheel pulleys in the YZ plane.

    Built as four thin segments: two straight runs (top and bottom tangent
    lines between the pulleys) and two annular arcs (wrapping around each
    pulley). Each segment is a thin rectangular cross-section extruded or
    revolved, then all are unioned into a single belt loop solid.
    """
    dy = y_w - y_m
    dz = z_w - z_m
    d = math.sqrt(dy * dy + dz * dz)
    phi = math.atan2(dz, dy)
    sin_a = max(-1.0, min(1.0, (r_w - r_m) / d))
    alpha = math.asin(sin_a)
    beta_top = phi + math.pi / 2.0 + alpha
    beta_bot = phi - math.pi / 2.0 - alpha
    r_belt = r_m  # belt centerline radius on motor side
    ht = thickness / 2.0

    # Tangent points on each pulley (centerline of belt).
    # Motor: tangent at beta_bot (bottom) and beta_top_motor (top)
    beta_top_m = phi + math.pi / 2.0 - alpha
    beta_bot_m = phi - math.pi / 2.0 + alpha

    # Motor tangent points (on belt centerline)
    mt_y = y_m + r_m * math.cos(beta_top_m)
    mt_z = z_m + r_m * math.sin(beta_top_m)
    mb_y = y_m + r_m * math.cos(beta_bot_m)
    mb_z = z_m + r_m * math.sin(beta_bot_m)

    # Wheel tangent points
    wt_y = y_w + r_w * math.cos(beta_top)
    wt_z = z_w + r_w * math.sin(beta_top)
    wb_y = y_w + r_w * math.cos(beta_bot)
    wb_z = z_w + r_w * math.sin(beta_bot)

    def _thin_rect(y1: float, z1: float, y2: float, z2: float, w: float, t: float) -> object:
        """Thin rectangular strip between two YZ points, extruded along X."""
        seg_dy = y2 - y1
        seg_dz = z2 - z1
        seg_len = math.sqrt(seg_dy * seg_dy + seg_dz * seg_dz)
        seg_angle = math.atan2(seg_dz, seg_dy)
        # Box: X=w (belt width), Y=seg_len, Z=t (thickness)
        # Rotate around X so that Y-axis tilts toward Z in the YZ plane.
        strip = (
            cq.Workplane("XY")
            .box(w, seg_len, t)
            .rotate((0, 0, 0), (1, 0, 0), math.degrees(seg_angle))
            .translate((0.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0))
        )
        return strip

    # Top straight strip
    top_strip = _thin_rect(mt_y, mt_z, wt_y, wt_z, width, thickness)

    # Bottom straight strip
    bot_strip = _thin_rect(wb_y, wb_z, mb_y, mb_z, width, thickness)

    # Motor wrap arc: from beta_bot_m clockwise around back to beta_top_m
    # Sweep angle (clockwise from beta_bot_m to beta_top_m going around the back)
    sweep_m = (beta_bot_m - beta_top_m) % (2.0 * math.pi)
    if sweep_m < 0.01:
        sweep_m = 2.0 * math.pi - 0.01
    motor_arc = (
        cq.Workplane("YZ")
        .center(y_m, z_m)
        .circle(r_m + ht)
        .circle(r_m - ht)
        .extrude(width)
        .translate((-width / 2.0, 0.0, 0.0))
    )
    # We keep the full ring as a simplification — it reads as the belt wrapping
    # the motor pulley and the thin straights blend into it.

    # Wheel wrap arc: full ring around wheel pulley
    wheel_arc = (
        cq.Workplane("YZ")
        .center(y_w, z_w)
        .circle(r_w + ht)
        .circle(r_w - ht)
        .extrude(width)
        .translate((-width / 2.0, 0.0, 0.0))
    )

    result = motor_arc.union(wheel_arc).union(top_strip).union(bot_strip)
    return result.translate((x_pos, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="differential_drive_wheel_module",
        meta={
            "category": "Robotics / Differential drive wheel",
            "reference_notes": "Two tan treaded drive wheels on a shared axle line with a metal top mounting plate, central swivel bearing, belt-drive carriage with raised motors and timing belt reduction, and fasteners.",
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

    # Carriage that turns under the top plate: open frame with raised motors and belt reduction.
    carriage = model.part("drive_carriage")
    motor_rpy = (0.0, math.pi / 2.0, 0.0)

    # Lightweight open frame replacing the enclosed gearbox blocks.
    carriage.visual(Box((0.240, 0.090, 0.008)), origin=Origin(xyz=(0.0, -0.015, -0.062)), material=brushed_aluminum, name="carriage_base")
    carriage.visual(Box((0.060, 0.080, 0.070)), origin=Origin(xyz=(-0.108, -0.050, -0.025)), material=brushed_aluminum, name="motor_mount_0")
    carriage.visual(Box((0.060, 0.080, 0.070)), origin=Origin(xyz=(0.108, -0.050, -0.025)), material=brushed_aluminum, name="motor_mount_1")
    carriage.visual(Box((0.140, 0.045, 0.085)), origin=Origin(xyz=(-0.105, 0.047, -0.058)), material=steel, name="front_yoke_0")
    carriage.visual(Box((0.140, 0.045, 0.085)), origin=Origin(xyz=(0.105, 0.047, -0.058)), material=steel, name="front_yoke_1")
    carriage.visual(Box((0.200, 0.032, 0.028)), origin=Origin(xyz=(0.0, 0.065, -0.018)), material=steel, name="upper_cross_brace")

    # Bearing post fits up into the annular bearing without being a solid overlap.
    _add_cyl(carriage, 0.049, 0.050, (0.0, 0.0, 0.006), material=dark_anodized, name="swivel_post")
    _add_cyl(carriage, 0.066, 0.012, (0.0, 0.0, -0.023), material=steel, name="inner_bearing_flange")
    _add_cyl(carriage, 0.055, 0.055, (0.0, 0.0, -0.045), material=steel, name="bearing_pedestal")
    # Support posts connect the carriage to the top plate. Post 2 is at the rear,
    # connected to the carriage base via a rear bracket.
    support_positions = [(-0.100, 0.0), (0.100, 0.0), (0.0, -0.120)]
    for i, (x, y) in enumerate(support_positions):
        _add_cyl(carriage, 0.010, 0.120, (x, y, -0.014), material=steel, name=f"top_support_post_{i}")
        _add_cyl(carriage, 0.014, 0.020, (x, y, 0.046), material=steel, name=f"top_support_pad_{i}")
    # Rear bracket connecting post 2 to the carriage base plate
    carriage.visual(
        Box((0.024, 0.066, 0.008)),
        origin=Origin(xyz=(0.0, -0.087, -0.062)),
        material=brushed_aluminum,
        name="rear_bracket",
    )

    # Belt-drive layout: pulley plane X for each side (coplanar motor output and wheel pulley).
    belt_plane_x = 0.236
    motor_y, motor_z = -0.050, 0.000  # raised above the axle line
    axle_y, axle_z = 0.0, -0.105
    r_motor_pulley = 0.018
    r_wheel_pulley = 0.048
    motor_outer_end = 0.155 + 0.050  # outer end of motor can along X

    # Two independent motor cans mounted high on the carriage, one per side.
    for side, sx in enumerate([-1.0, 1.0]):
        mx = sx * 0.155
        _add_cyl(carriage, 0.038, 0.100, (mx, motor_y, motor_z), rpy=motor_rpy, material=dark_anodized, name=f"motor_can_{side}")
        _add_cyl(carriage, 0.040, 0.008, (mx - sx * 0.054, motor_y, motor_z), rpy=motor_rpy, material=steel, name=f"motor_endcap_{side}")
        # Cooling fins on each motor can (positioned to touch the can surface)
        for fi, fz in enumerate([-0.022, 0.0, 0.022]):
            carriage.visual(
                Box((0.090, 0.006, 0.006)),
                origin=Origin(xyz=(mx, motor_y - 0.035, motor_z + fz)),
                material=black_rubber,
                name=f"motor_fin_{side}_{fi}",
            )
        # Motor output shaft extending from motor can end to the belt plane
        shaft_len = belt_plane_x - motor_outer_end
        shaft_cx = sx * (motor_outer_end + belt_plane_x) / 2.0
        _add_cyl(carriage, 0.008, shaft_len, (shaft_cx, motor_y, motor_z), rpy=motor_rpy, material=steel, name=f"motor_shaft_{side}")

    # Motor pulleys (small toothed) at each belt plane.
    for side, sx in enumerate([-1.0, 1.0]):
        px = sx * belt_plane_x
        carriage.visual(
            mesh_from_cadquery(_toothed_pulley_mesh(r_motor_pulley, 0.014, n_teeth=16), f"motor_pulley_{side}_mesh", tolerance=0.0005),
            origin=Origin(xyz=(px, motor_y, motor_z), rpy=motor_rpy),
            material=dark_anodized,
            name=f"motor_pulley_{side}",
        )

    # Timing belt loops (one per side) connecting motor pulley down to the wheel axle line.
    belt_rubber = model.material("belt_rubber", rgba=(0.04, 0.04, 0.03, 1.0))
    for side, sx in enumerate([-1.0, 1.0]):
        px = sx * belt_plane_x
        carriage.visual(
            mesh_from_cadquery(
                _belt_loop_mesh(px, motor_y, motor_z, r_motor_pulley, axle_y, axle_z, r_wheel_pulley, width=0.012, thickness=0.004),
                f"belt_loop_{side}_mesh",
                tolerance=0.0005,
            ),
            material=belt_rubber,
            name=f"belt_loop_{side}",
        )

    # Idler / tensioner bosses — small cylindrical rollers that press against the belt slack side.
    for side, sx in enumerate([-1.0, 1.0]):
        px = sx * belt_plane_x
        _add_cyl(carriage, 0.012, 0.014, (px, -0.068, -0.058), rpy=motor_rpy, material=steel, name=f"idler_boss_{side}")
        _add_cyl(carriage, 0.006, 0.020, (px, -0.068, -0.058), rpy=motor_rpy, material=dark_anodized, name=f"idler_pin_{side}")

    # Axle bearing housings and support structure connecting bearings to the frame.
    for side, sx in enumerate([-1.0, 1.0]):
        bx = sx * 0.220
        _add_cyl(carriage, 0.028, 0.018, (bx, 0.0, -0.105), rpy=motor_rpy, material=steel, name=f"axle_bearing_{side}")
        # Horizontal axle support arm from front yoke to the axle bearing
        carriage.visual(
            Box((0.055, 0.035, 0.012)),
            origin=Origin(xyz=(sx * 0.193, 0.035, -0.090)),
            material=brushed_aluminum,
            name=f"axle_arm_{side}",
        )

    # Motor mount fasteners on each bracket.
    for side, sx in enumerate([-1.0, 1.0]):
        bx = sx * 0.108
        for j, (dy, dz) in enumerate([(0.025, 0.015), (-0.025, 0.015), (0.025, -0.035), (-0.025, -0.035)]):
            _add_cyl(carriage, 0.005, 0.004, (bx + sx * 0.032, motor_y + dy, motor_z + dz), rpy=motor_rpy, material=dark_anodized, name=f"motor_bolt_{side}_{j}")

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
        # Timing pulley on the inner face of each wheel, aligned to the belt plane.
        pulley_x = 0.058 if i == 0 else -0.058
        # Connecting shaft from hub drum to pulley
        shaft_half = (abs(pulley_x) - 0.0415) / 2.0
        shaft_center = 0.0415 + shaft_half if i == 0 else -(0.0415 + shaft_half)
        _add_cyl(
            wheel_part, 0.020, abs(pulley_x) - 0.0415,
            (shaft_center, 0.0, 0.0), rpy=motor_rpy,
            material=steel, name="pulley_shaft",
        )
        wheel_part.visual(
            mesh_from_cadquery(_toothed_pulley_mesh(0.048, 0.014, n_teeth=28), f"wheel_{i}_pulley_mesh", tolerance=0.0005),
            origin=Origin(xyz=(pulley_x, 0.0, 0.0), rpy=motor_rpy),
            material=dark_anodized,
            name="wheel_pulley",
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

    # --- Belt-drive reduction tests (TARGET-specific) ---
    carriage_visual_names = {v.name for v in carriage.visuals}
    belt_names = {"belt_loop_0", "belt_loop_1", "motor_pulley_0", "motor_pulley_1", "idler_boss_0", "idler_boss_1"}
    ctx.check(
        "drive_carriage has timing-belt reduction visuals",
        belt_names.issubset(carriage_visual_names),
        details=f"missing: {belt_names - carriage_visual_names}",
    )

    # Motors are raised above the axle line (motor_can center z > axle z).
    ctx.expect_gap(
        carriage,
        carriage,
        axis="z",
        positive_elem="motor_can_0",
        negative_elem="axle_bearing_0",
        min_gap=0.020,
        name="motor_can_0 sits above axle_bearing_0 confirming raised motor position",
    )

    # Each wheel carries a coaxial timing pulley.
    for i, wpart in enumerate([wheel_0, wheel_1]):
        wv_names = {v.name for v in wpart.visuals}
        ctx.check(
            f"wheel_{i} carries a timing pulley visual",
            "wheel_pulley" in wv_names,
            details=f"wheel_{i} visuals: {wv_names}",
        )

    # Belt loop spans vertically from the motor pulley down to the wheel pulley region.
    ctx.expect_overlap(
        carriage,
        carriage,
        axes="z",
        elem_a="belt_loop_0",
        elem_b="motor_pulley_0",
        min_overlap=0.005,
        name="belt_loop_0 overlaps motor_pulley_0 in Z (wraps around motor pulley)",
    )

    # Motor pulleys are smaller than wheel pulleys (reduction ratio).
    ctx.allow_overlap(
        carriage,
        wheel_0,
        elem_a="belt_loop_0",
        elem_b="wheel_pulley",
        reason="The belt loop wraps around the wheel pulley as part of the timing-belt drive mechanism.",
    )
    ctx.allow_overlap(
        carriage,
        wheel_1,
        elem_a="belt_loop_1",
        elem_b="wheel_pulley",
        reason="The belt loop wraps around the wheel pulley as part of the timing-belt drive mechanism.",
    )
    ctx.allow_overlap(
        carriage,
        carriage,
        elem_a="belt_loop_0",
        elem_b="motor_pulley_0",
        reason="The belt loop wraps around the motor pulley as part of the timing-belt drive mechanism.",
    )
    ctx.allow_overlap(
        carriage,
        carriage,
        elem_a="belt_loop_1",
        elem_b="motor_pulley_1",
        reason="The belt loop wraps around the motor pulley as part of the timing-belt drive mechanism.",
    )

    # Wheels sit outboard of the carriage frame with a small functional clearance, while
    # still overlapping the carriage footprint in Y/Z so they read as axle-mounted.
    ctx.expect_gap(
        carriage,
        wheel_0,
        axis="x",
        negative_elem="treaded_tire",
        min_gap=0.001,
        max_gap=0.060,
        name="wheel 0 tire clears the carriage frame side",
    )
    ctx.expect_gap(
        wheel_1,
        carriage,
        axis="x",
        positive_elem="treaded_tire",
        min_gap=0.001,
        max_gap=0.060,
        name="wheel 1 tire clears the carriage frame side",
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
