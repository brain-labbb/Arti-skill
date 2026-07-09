from __future__ import annotations

from math import cos, pi, sin

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
    LatheGeometry,
    LoftSection,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SectionLoftSpec,
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
    section_loft,
    tube_from_spline_points,
)


def _circle_xyz(cx: float, cy: float, cz: float, radius: float, count: int = 24) -> list[tuple[float, float, float]]:
    """Return a closed circular loop in 3D at height cz."""
    return [
        (cx + radius * cos(2.0 * pi * i / count), cy + radius * sin(2.0 * pi * i / count), cz)
        for i in range(count)
    ]


def _conical_hopper_mesh() -> MeshGeometry:
    """Round-conical funnel hopper: circular top rim narrowing to a central cone throat."""
    # Outer surface profile: (radius, z) from top to throat.
    outer_profile = [
        (0.38, 0.84),   # top rim
        (0.37, 0.78),   # upper body
        (0.35, 0.72),   # shoulder
        (0.34, 0.63),   # mid body
        (0.28, 0.54),   # taper start
        (0.22, 0.46),   # lower taper
        (0.16, 0.40),   # near throat
        (0.12, 0.36),   # throat
    ]
    wall = 0.028
    inner_profile = [
        (0.38 - wall, 0.84),
        (0.37 - wall, 0.78),
        (0.35 - wall, 0.72),
        (0.34 - wall, 0.63),
        (0.28 - wall, 0.54),
        (0.22 - wall, 0.46),
        (0.16 - wall, 0.40),
        (0.12 - wall, 0.36),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=36,
        start_cap="flat",
        end_cap="flat",
    )
    return shell


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
    """Open sloped metering chute with circular inlet from the round hopper throat."""
    n_pts = 16
    # Circular inlet at the hopper throat, extending up to contact the hopper shell.
    inlet_circle = _circle_xyz(0.0, -0.055, 0.360, 0.10, count=n_pts)
    # Wider flattened outlet over the spinner plate.
    outlet_half_w = 0.16
    outlet_y = -0.185
    outlet_z = 0.303
    outlet_pts = [
        (
            outlet_half_w * cos(2.0 * pi * i / n_pts),
            outlet_y + 0.06 * sin(2.0 * pi * i / n_pts),
            outlet_z,
        )
        for i in range(n_pts)
    ]
    # Build as section loft with two sections: inlet circle and outlet ellipse.
    inlet_section = LoftSection(points=tuple(tuple(p) for p in inlet_circle))
    outlet_section = LoftSection(points=tuple(tuple(p) for p in outlet_pts))
    return section_loft(
        SectionLoftSpec(
            sections=(inlet_section, outlet_section),
            cap=False,
            solid=False,
            ruled=True,
        )
    )


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

    # Round-conical funnel hopper and its circular reinforcing rings.
    spreader.visual(
        mesh_from_geometry(_conical_hopper_mesh(), "hopper_shell"),
        material=black_plastic,
        name="hopper_shell",
    )
    spreader.visual(
        mesh_from_geometry(TorusGeometry(0.38, 0.018, radial_segments=14, tubular_segments=36), "top_rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=black_plastic,
        name="top_rolled_rim",
    )
    spreader.visual(
        mesh_from_geometry(TorusGeometry(0.34, 0.016, radial_segments=14, tubular_segments=32), "middle_seam_rib"),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=black_plastic,
        name="middle_seam_rib",
    )
    spreader.visual(
        mesh_from_geometry(TorusGeometry(0.16, 0.018, radial_segments=14, tubular_segments=24), "lower_support_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=satin_black,
        name="lower_support_ring",
    )

    # Orange brand-like hopper markings sit on the curved round surface.
    spreader.visual(
        Box((0.145, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, -0.355, 0.705)),
        material=orange,
        name="orange_brand_block",
    )
    for i in range(3):
        spreader.visual(
            Box((0.095 - i * 0.015, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, -0.340, 0.672 - i * 0.014)),
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
        Box((0.340, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.020, 0.345)),
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
    spinner_joint = object_model.get_articulation("spreader_to_spinner")
    lever_joint = object_model.get_articulation("spreader_to_lever")
    gate_joint = object_model.get_articulation("lever_to_gate")

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
    # Variant-specific: hopper_shell must be round-conical (circular cross-section).
    hopper_aabb = ctx.part_element_world_aabb(spreader, elem="hopper_shell")
    if hopper_aabb is not None:
        dx = hopper_aabb[1][0] - hopper_aabb[0][0]
        dy = hopper_aabb[1][1] - hopper_aabb[0][1]
        ctx.check(
            "hopper_shell is round-conical (circular XY footprint)",
            abs(dx - dy) < 0.04 and dx > 0.50,
            details=f"hopper_shell AABB dx={dx:.4f}, dy={dy:.4f} (should be roughly equal for circular section)",
        )
    else:
        ctx.fail("hopper_shell is round-conical (circular XY footprint)", "could not measure hopper_shell AABB")
    # Chute inlet should be circular (equal X/Y extents).
    chute_aabb = ctx.part_element_world_aabb(spreader, elem="chute_tray")
    if chute_aabb is not None:
        cdx = chute_aabb[1][0] - chute_aabb[0][0]
        cdy = chute_aabb[1][1] - chute_aabb[0][1]
        ctx.check(
            "chute_tray inlet is circular",
            cdx > 0.10 and cdy > 0.08,
            details=f"chute_tray AABB dx={cdx:.4f}, dy={cdy:.4f}",
        )
    ctx.check(
        "key seed spreader subassemblies are present",
        all(part is not None for part in (spreader, spinner, gate, lever, wheel_0, wheel_1)),
        details="expected hopper/frame, two wheels, spinner, flow gate, and control lever",
    )
    ctx.check(
        "visible non-fixed mechanisms are authored",
        spinner_joint.articulation_type == ArticulationType.CONTINUOUS
        and lever_joint.articulation_type == ArticulationType.REVOLUTE
        and object_model.get_articulation("lever_to_gate").articulation_type == ArticulationType.PRISMATIC,
        details="spinner must rotate and lever/gate must move for seed metering",
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
