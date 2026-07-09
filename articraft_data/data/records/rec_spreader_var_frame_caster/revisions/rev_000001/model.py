from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _loop_vertices(mesh: MeshGeometry, profile: list[tuple[float, float]], z: float) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y in profile]


def _rect_shell_mesh() -> MeshGeometry:
    """Thin open plastic hopper with a rectangular tub and tapered lower funnel."""
    mesh = MeshGeometry()
    # Top tub, shoulder, lower funnel, and bottom metering throat.
    sections = [
        (0.84, 0.76, 0.54, 0.070),
        (0.63, 0.72, 0.50, 0.060),
        (0.45, 0.43, 0.30, 0.045),
        (0.36, 0.23, 0.16, 0.030),
    ]
    wall = 0.030
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []
    for z, width, depth, radius in sections:
        outer = rounded_rect_profile(width, depth, radius, corner_segments=7)
        inner = rounded_rect_profile(
            max(width - 2.0 * wall, 0.04),
            max(depth - 2.0 * wall, 0.04),
            max(radius - wall * 0.55, 0.012),
            corner_segments=7,
        )
        # The inside lip is slightly below the outside rim so the wall reads hollow.
        inner_z = z - (0.030 if z == sections[0][0] else 0.018)
        outer_loops.append(_loop_vertices(mesh, outer, z))
        inner_loops.append(_loop_vertices(mesh, inner, inner_z))

    count = len(outer_loops[0])
    for i in range(len(sections) - 1):
        for j in range(count):
            n = (j + 1) % count
            _add_quad(mesh, outer_loops[i][j], outer_loops[i][n], outer_loops[i + 1][n], outer_loops[i + 1][j])
            # Reverse winding for the inside wall.
            _add_quad(mesh, inner_loops[i][n], inner_loops[i][j], inner_loops[i + 1][j], inner_loops[i + 1][n])

    # Rounded top rim cap and lower outlet ring cap.
    for j in range(count):
        n = (j + 1) % count
        _add_quad(mesh, outer_loops[0][n], outer_loops[0][j], inner_loops[0][j], inner_loops[0][n])
        _add_quad(mesh, outer_loops[-1][j], outer_loops[-1][n], inner_loops[-1][n], inner_loops[-1][j])
    return mesh


def _hopper_loop_mesh(width: float, depth: float, z: float, radius: float, tube_radius: float, name: str):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    points = [(x, y, z) for x, y in profile]
    tube = tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=5,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )
    return mesh_from_geometry(tube, name)


def _tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str, *, samples: int = 8):
    tube = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(tube, name)


def _chute_mesh() -> MeshGeometry:
    """Open sloped metering chute leading from the gate to the spinning plate."""
    mesh = MeshGeometry()
    # Four stations: inlet high and rearward, outlet lower over the spinner.
    half_width_in = 0.13
    half_width_out = 0.18
    y_in, z_in = -0.055, 0.342
    y_out, z_out = -0.185, 0.303
    wall = 0.055
    pts = {
        "bi_l": mesh.add_vertex(-half_width_in, y_in, z_in),
        "bi_r": mesh.add_vertex(half_width_in, y_in, z_in),
        "bo_l": mesh.add_vertex(-half_width_out, y_out, z_out),
        "bo_r": mesh.add_vertex(half_width_out, y_out, z_out),
        "li_t": mesh.add_vertex(-half_width_in, y_in, z_in + wall),
        "lo_t": mesh.add_vertex(-half_width_out, y_out, z_out + wall * 0.72),
        "ri_t": mesh.add_vertex(half_width_in, y_in, z_in + wall),
        "ro_t": mesh.add_vertex(half_width_out, y_out, z_out + wall * 0.72),
    }
    # Bottom and two raised side walls.
    _add_quad(mesh, pts["bi_l"], pts["bi_r"], pts["bo_r"], pts["bo_l"])
    _add_quad(mesh, pts["li_t"], pts["bi_l"], pts["bo_l"], pts["lo_t"])
    _add_quad(mesh, pts["bi_r"], pts["ri_t"], pts["ro_t"], pts["bo_r"])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_seed_spreader",
        meta={"category": "Agricultural", "small_class": "Seed spreader"},
    )

    black_plastic = Material("molded_black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))
    satin_black = Material("painted_black_steel", rgba=(0.02, 0.02, 0.018, 1.0))
    rubber = Material("textured_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    galvanized = Material("galvanized_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    white_rim = Material("white_painted_rim", rgba=(0.88, 0.86, 0.82, 1.0))
    orange = Material("orange_hopper_label", rgba=(1.0, 0.34, 0.02, 1.0))
    dark_hardware = Material("dark_fastener_heads", rgba=(0.01, 0.01, 0.009, 1.0))

    spreader = model.part("spreader")

    # Hollow molded hopper and its rolled/reinforced edges.
    spreader.visual(
        mesh_from_geometry(_rect_shell_mesh(), "hopper_shell"),
        material=black_plastic,
        name="hopper_shell",
    )
    spreader.visual(
        _hopper_loop_mesh(0.79, 0.57, 0.845, 0.080, 0.018, "top_rolled_rim"),
        material=black_plastic,
        name="top_rolled_rim",
    )
    spreader.visual(
        _hopper_loop_mesh(0.73, 0.51, 0.615, 0.060, 0.016, "middle_seam_rib"),
        material=black_plastic,
        name="middle_seam_rib",
    )
    spreader.visual(
        _hopper_loop_mesh(0.35, 0.25, 0.400, 0.035, 0.018, "lower_support_ring"),
        material=satin_black,
        name="lower_support_ring",
    )

    # Orange brand-like hopper markings are raised decals, not just material color.
    spreader.visual(
        Box((0.145, 0.004, 0.050)),
        origin=Origin(xyz=(-0.145, -0.255, 0.705)),
        material=orange,
        name="orange_brand_block",
    )
    for i in range(3):
        spreader.visual(
            Box((0.095 - i * 0.015, 0.014, 0.006)),
            origin=Origin(xyz=(-0.145, -0.255, 0.672 - i * 0.014)),
            material=orange,
            name=f"label_line_{i}",
        )

    # Welded/bolted steel chassis, axle, handle, and support tubes.
    spreader.visual(
        Cylinder(radius=0.016, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="wheel_axle",
    )
    spreader.visual(
        Cylinder(radius=0.052, length=0.135),
        origin=Origin(xyz=(0.0, -0.010, 0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="gearbox_bulge",
    )
    spreader.visual(
        Box((0.115, 0.090, 0.115)),
        origin=Origin(xyz=(0.0, -0.020, 0.120)),
        material=satin_black,
        name="gearbox_case",
    )
    spreader.visual(
        _tube_mesh([(-0.31, 0.01, 0.180), (-0.285, -0.08, 0.285), (-0.255, -0.18, 0.430), (-0.225, -0.18, 0.555)], 0.014, "frame_side_0"),
        material=satin_black,
        name="frame_side_0",
    )
    spreader.visual(
        _tube_mesh([(0.31, 0.01, 0.180), (0.285, -0.08, 0.285), (0.255, -0.18, 0.430), (0.225, -0.18, 0.555)], 0.014, "frame_side_1"),
        material=satin_black,
        name="frame_side_1",
    )
    spreader.visual(
        _tube_mesh([(-0.26, 0.055, 0.200), (-0.18, 0.160, 0.360), (-0.10, 0.190, 0.580)], 0.013, "rear_brace_0"),
        material=satin_black,
        name="rear_brace_0",
    )
    spreader.visual(
        _tube_mesh([(0.26, 0.055, 0.200), (0.18, 0.160, 0.360), (0.10, 0.190, 0.580)], 0.013, "rear_brace_1"),
        material=satin_black,
        name="rear_brace_1",
    )
    spreader.visual(
        _tube_mesh([(0.0, 0.140, 0.565), (0.0, 0.350, 0.820), (0.0, 0.560, 1.085)], 0.019, "handle_stem"),
        material=satin_black,
        name="handle_stem",
    )
    spreader.visual(
        Cylinder(radius=0.017, length=0.77),
        origin=Origin(xyz=(0.0, 0.625, 1.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="handlebar_tube",
    )
    spreader.visual(
        Cylinder(radius=0.031, length=0.270),
        origin=Origin(xyz=(-0.300, 0.625, 1.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip_0",
    )
    spreader.visual(
        Cylinder(radius=0.031, length=0.270),
        origin=Origin(xyz=(0.300, 0.625, 1.120), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip_1",
    )
    spreader.visual(
        Box((0.180, 0.060, 0.075)),
        origin=Origin(xyz=(0.0, 0.585, 1.085)),
        material=satin_black,
        name="handle_control_bracket",
    )

    # Metering chute, gate guide rails, and the visible cable route from the handle control to the gate.
    spreader.visual(
        mesh_from_geometry(_chute_mesh(), "chute_tray"),
        material=satin_black,
        name="chute_tray",
    )
    for i, x in enumerate((-0.165, 0.165)):
        spreader.visual(
            Box((0.014, 0.270, 0.016)),
            origin=Origin(xyz=(x, -0.065, 0.345)),
            material=satin_black,
            name=f"flow_gate_side_rail_{i}",
        )
    spreader.visual(
        Box((0.285, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.040, 0.345)),
        material=satin_black,
        name="flow_gate_rear_stop",
    )
    spreader.visual(
        Box((0.030, 0.128, 0.050)),
        origin=Origin(xyz=(0.0, -0.095, 0.230)),
        material=satin_black,
        name="spinner_bearing_arm",
    )
    spreader.visual(
        mesh_from_geometry(TorusGeometry(0.015, 0.004, radial_segments=20, tubular_segments=12), "spinner_bearing_ring"),
        origin=Origin(xyz=(0.0, -0.170, 0.245)),
        material=galvanized,
        name="spinner_bearing_ring",
    )
    spreader.visual(
        _tube_mesh([(0.085, 0.575, 1.055), (0.105, 0.335, 0.820), (0.080, 0.105, 0.560), (0.040, -0.030, 0.382)], 0.0035, "control_cable", samples=10),
        material=galvanized,
        name="control_cable",
    )
    for i, x in enumerate((-0.391, 0.391)):
        spreader.visual(
            Cylinder(radius=0.130, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"axle_spacer_{i}",
        )

    # Small dark hardware: hopper/ring bolts, axle nuts, and bracket screws.
    for idx, (x, y, z) in enumerate(
        [
            (-0.250, -0.180, 0.548),
            (0.250, -0.180, 0.548),
            (-0.045, 0.585, 1.128),
            (0.045, 0.585, 1.128),
        ]
    ):
        spreader.visual(
            Sphere(radius=0.012 if idx < 4 else 0.010),
            origin=Origin(xyz=(x, y, z)),
            material=dark_hardware,
            name=f"fastener_{idx}",
        )

    # Front-center caster fork assembly: brace tube, mount plate, fork stem, legs, and axle.
    spreader.visual(
        _tube_mesh(
            [(0.0, -0.065, 0.120), (0.0, -0.145, 0.140), (0.0, -0.210, 0.170)],
            0.013,
            "caster_front_brace",
        ),
        material=satin_black,
        name="caster_front_brace",
    )
    spreader.visual(
        Box((0.055, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.215, 0.170)),
        material=satin_black,
        name="caster_mount_plate",
    )
    spreader.visual(
        _tube_mesh(
            [(0.0, -0.215, 0.161), (0.0, -0.238, 0.160)],
            0.011,
            "caster_fork_stem",
        ),
        material=satin_black,
        name="caster_fork_stem",
    )
    spreader.visual(
        Cylinder(radius=0.010, length=0.076),
        origin=Origin(xyz=(0.0, -0.240, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="caster_fork_crown",
    )
    spreader.visual(
        _tube_mesh(
            [(-0.036, -0.242, 0.158), (-0.042, -0.246, 0.110), (-0.046, -0.248, 0.070)],
            0.009,
            "caster_fork_leg_0",
        ),
        material=satin_black,
        name="caster_fork_leg_0",
    )
    spreader.visual(
        _tube_mesh(
            [(0.036, -0.242, 0.158), (0.042, -0.246, 0.110), (0.046, -0.248, 0.070)],
            0.009,
            "caster_fork_leg_1",
        ),
        material=satin_black,
        name="caster_fork_leg_1",
    )
    spreader.visual(
        Cylinder(radius=0.005, length=0.100),
        origin=Origin(xyz=(0.0, -0.248, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="caster_axle",
    )
    spreader.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(-0.052, -0.248, 0.070)),
        material=dark_hardware,
        name="caster_axle_nut_0",
    )
    spreader.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.052, -0.248, 0.070)),
        material=dark_hardware,
        name="caster_axle_nut_1",
    )

    # Shared detailed utility tire and white rim meshes.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.185,
            0.082,
            inner_radius=0.128,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.055),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.52),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
            sidewall=TireSidewall(style="square", bulge=0.035),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "utility_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.128,
            0.060,
            rim=WheelRim(inner_radius=0.080, flange_height=0.010, flange_thickness=0.004, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.034,
                width=0.038,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.052, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.040),
        ),
        "white_wheel_rim",
    )

    for i, x in enumerate((-0.435, 0.435)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(rim_mesh, material=white_rim, name="rim")
        wheel.visual(
            Cylinder(radius=0.020, length=0.066),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_hardware,
            name="bearing_sleeve",
        )
        model.articulation(
            f"spreader_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=spreader,
            child=wheel,
            origin=Origin(xyz=(x, 0.0, 0.180)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=12.0),
        )

    # Front-center caster wheel: smaller tire, rim, and bearing sleeve.
    caster = model.part("caster_wheel")
    caster.visual(
        mesh_from_geometry(
            TireGeometry(
                0.075,
                0.032,
                inner_radius=0.050,
                carcass=TireCarcass(belt_width_ratio=0.60, sidewall_bulge=0.030),
                tread=TireTread(style="block", depth=0.005, count=14, land_ratio=0.50),
                sidewall=TireSidewall(style="square", bulge=0.020),
                shoulder=TireShoulder(width=0.004, radius=0.003),
            ),
            "caster_tire",
        ),
        material=rubber,
        name="caster_tire",
    )
    caster.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.050,
                0.025,
                rim=WheelRim(
                    inner_radius=0.032,
                    flange_height=0.005,
                    flange_thickness=0.003,
                    bead_seat_depth=0.003,
                ),
                hub=WheelHub(
                    radius=0.014,
                    width=0.018,
                    cap_style="flat",
                    bolt_pattern=BoltPattern(count=4, circle_diameter=0.022, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="split_y", count=4, thickness=0.003, window_radius=0.009),
                bore=WheelBore(style="round", diameter=0.012),
            ),
            "caster_rim",
        ),
        material=white_rim,
        name="caster_rim",
    )
    caster.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_hardware,
        name="caster_bearing",
    )
    model.articulation(
        "spreader_to_caster",
        ArticulationType.CONTINUOUS,
        parent=spreader,
        child=caster,
        origin=Origin(xyz=(0.0, -0.248, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )

    spinner = model.part("spinner")
    spinner.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.155,
                0.032,
                4,
                thickness=0.014,
                blade_pitch_deg=8.0,
                blade_sweep_deg=4.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=4.0, camber=0.04),
                hub=FanRotorHub(style="flat", bore_diameter=0.010),
            ),
            "spinning_plate",
        ),
        material=satin_black,
        name="spinning_plate",
    )
    spinner.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=galvanized,
        name="spinner_shaft",
    )
    model.articulation(
        "spreader_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=spreader,
        child=spinner,
        origin=Origin(xyz=(0.0, -0.170, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=20.0),
    )

    lever = model.part("control_lever")
    lever.visual(
        Cylinder(radius=0.007, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=galvanized,
        name="lever_arm",
    )
    lever.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.052,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=16, depth=0.0022),
            ),
            "lever_knob",
        ),
        # KnobGeometry is aligned to local Z; rotate it so it lies like a small
        # hand grip at the end of the metering lever.
        origin=Origin(xyz=(0.0, 0.0, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="lever_knob",
    )
    model.articulation(
        "spreader_to_lever",
        ArticulationType.REVOLUTE,
        parent=spreader,
        child=lever,
        origin=Origin(xyz=(0.097, 0.580, 1.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=0.65),
    )

    gate = model.part("flow_gate")
    gate.visual(
        Box((0.285, 0.180, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="slide_plate",
    )
    model.articulation(
        "lever_to_gate",
        ArticulationType.PRISMATIC,
        parent=spreader,
        child=gate,
        origin=Origin(xyz=(0.0, -0.040, 0.334)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.130),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spreader = object_model.get_part("spreader")
    spinner = object_model.get_part("spinner")
    gate = object_model.get_part("flow_gate")
    lever = object_model.get_part("control_lever")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    caster = object_model.get_part("caster_wheel")
    spinner_joint = object_model.get_articulation("spreader_to_spinner")
    lever_joint = object_model.get_articulation("spreader_to_lever")
    gate_joint = object_model.get_articulation("lever_to_gate")
    caster_joint = object_model.get_articulation("spreader_to_caster")

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            wheel,
            spreader,
            elem_a="bearing_sleeve",
            elem_b="wheel_axle",
            reason="The wheel bearing sleeve is intentionally modeled as captured around the through axle.",
        )
        ctx.expect_overlap(
            wheel,
            spreader,
            axes="x",
            min_overlap=0.035,
            elem_a="bearing_sleeve",
            elem_b="wheel_axle",
            name=f"{wheel.name} bearing stays retained on axle",
        )
        ctx.expect_overlap(
            wheel,
            spreader,
            axes="yz",
            min_overlap=0.030,
            elem_a="bearing_sleeve",
            elem_b="wheel_axle",
            name=f"{wheel.name} axle passes through bearing bore",
        )

    ctx.allow_overlap(
        caster,
        spreader,
        elem_a="caster_bearing",
        elem_b="caster_axle",
        reason="The caster bearing sleeve is intentionally modeled as captured around the caster axle.",
    )
    ctx.expect_overlap(
        caster,
        spreader,
        axes="x",
        min_overlap=0.012,
        elem_a="caster_bearing",
        elem_b="caster_axle",
        name="caster bearing stays retained on caster axle",
    )
    ctx.expect_overlap(
        caster,
        spreader,
        axes="yz",
        min_overlap=0.008,
        elem_a="caster_bearing",
        elem_b="caster_axle",
        name="caster axle passes through bearing bore",
    )

    ctx.allow_overlap(
        spinner,
        spreader,
        elem_a="spinner_shaft",
        elem_b="spinner_bearing_ring",
        reason="The spinner shaft is intentionally captured by the stationary bearing ring.",
    )
    ctx.expect_overlap(
        spinner,
        spreader,
        axes="z",
        min_overlap=0.006,
        elem_a="spinner_shaft",
        elem_b="spinner_bearing_ring",
        name="spinner shaft remains captured in bearing ring",
    )
    ctx.expect_overlap(
        spinner,
        spreader,
        axes="xy",
        min_overlap=0.020,
        elem_a="spinner_shaft",
        elem_b="spinner_bearing_ring",
        name="spinner shaft is centered in bearing ring",
    )
    ctx.allow_overlap(
        gate,
        spreader,
        elem_a="slide_plate",
        elem_b="chute_tray",
        reason="The sliding flow gate is intentionally captured in the simplified chute slot.",
    )

    ctx.check(
        "classified as seed spreader",
        object_model.name == "agricultural_seed_spreader"
        and object_model.meta.get("category") == "Agricultural"
        and object_model.meta.get("small_class") == "Seed spreader",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "key seed spreader subassemblies are present",
        all(part is not None for part in (spreader, spinner, gate, lever, wheel_0, wheel_1, caster)),
        details="expected hopper/frame, two drive wheels, caster wheel, spinner, flow gate, and control lever",
    )
    ctx.check(
        "caster_wheel has continuous roll joint",
        caster_joint is not None
        and caster_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="front-center caster_wheel must rotate freely on its axle via a CONTINUOUS joint",
    )
    ctx.check(
        "visible non-fixed mechanisms are authored",
        spinner_joint.articulation_type == ArticulationType.CONTINUOUS
        and caster_joint.articulation_type == ArticulationType.CONTINUOUS
        and lever_joint.articulation_type == ArticulationType.REVOLUTE
        and object_model.get_articulation("lever_to_gate").articulation_type == ArticulationType.PRISMATIC,
        details="spinner and caster must rotate, lever/gate must move for seed metering",
    )
    ctx.expect_gap(
        spreader,
        spinner,
        axis="z",
        min_gap=0.010,
        max_gap=0.060,
        positive_elem="chute_tray",
        negative_elem="spinning_plate",
        name="chute feeds above spinning diffuser plate",
    )
    ctx.expect_overlap(
        spinner,
        spreader,
        axes="xy",
        min_overlap=0.080,
        elem_a="spinning_plate",
        elem_b="chute_tray",
        name="spinner sits under seed chute footprint",
    )
    ctx.expect_overlap(
        gate,
        spreader,
        axes="xy",
        min_overlap=0.070,
        elem_a="slide_plate",
        elem_b="chute_tray",
        name="sliding flow gate covers the chute inlet area",
    )

    rest_gate = ctx.part_world_position(gate)
    rest_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    with ctx.pose({lever_joint: 0.60, gate_joint: 0.115, spinner_joint: 1.2}):
        moved_gate = ctx.part_world_position(gate)
        moved_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
        ctx.expect_overlap(
            gate,
            spreader,
            axes="x",
            min_overlap=0.120,
            elem_a="slide_plate",
            elem_b="chute_tray",
            name="opened gate remains centered between chute rails",
        )

    def _aabb_center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "control pose opens flow gate",
        rest_gate is not None and moved_gate is not None and moved_gate[1] < rest_gate[1] - 0.095,
        details=f"rest_gate={rest_gate}, moved_gate={moved_gate}",
    )
    ctx.check(
        "control lever visibly pivots",
        _aabb_center_z(rest_knob_aabb) is not None
        and _aabb_center_z(moved_knob_aabb) is not None
        and abs(_aabb_center_z(moved_knob_aabb) - _aabb_center_z(rest_knob_aabb)) > 0.025,
        details=f"rest_knob_aabb={rest_knob_aabb}, moved_knob_aabb={moved_knob_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
