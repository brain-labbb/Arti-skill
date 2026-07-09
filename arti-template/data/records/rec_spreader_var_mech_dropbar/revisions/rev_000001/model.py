from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    """Wide rectangular open plastic hopper with a long drop-box profile and tapered lower throat."""
    mesh = MeshGeometry()
    # Widened for a long drop-box rectangle (companion variation 5).
    # Each entry is (z_height, x_width, y_depth, corner_radius).
    sections = [
        (0.84, 0.88, 0.54, 0.040),
        (0.63, 0.84, 0.50, 0.038),
        (0.45, 0.76, 0.38, 0.032),
        (0.36, 0.68, 0.18, 0.025),
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
        inner_z = z - (0.030 if z == sections[0][0] else 0.018)
        outer_loops.append(_loop_vertices(mesh, outer, z))
        inner_loops.append(_loop_vertices(mesh, inner, inner_z))

    count = len(outer_loops[0])
    for i in range(len(sections) - 1):
        for j in range(count):
            n = (j + 1) % count
            _add_quad(mesh, outer_loops[i][j], outer_loops[i][n], outer_loops[i + 1][n], outer_loops[i + 1][j])
            _add_quad(mesh, inner_loops[i][n], inner_loops[i][j], inner_loops[i + 1][j], inner_loops[i + 1][n])

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
    """Wide distribution tray from the flow gate down to the drop bar metering holes."""
    mesh = MeshGeometry()
    half_width_in = 0.14
    half_width_out = 0.28
    y_in, z_in = -0.04, 0.338
    y_out, z_out = -0.12, 0.275
    wall = 0.035
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
    _add_quad(mesh, pts["bi_l"], pts["bi_r"], pts["bo_r"], pts["bo_l"])
    _add_quad(mesh, pts["li_t"], pts["bi_l"], pts["bo_l"], pts["lo_t"])
    _add_quad(mesh, pts["bi_r"], pts["ri_t"], pts["ro_t"], pts["bo_r"])
    return mesh


# ---------------------------------------------------------------------------
# Shared drop-bar constants (used by both build and tests)
# ---------------------------------------------------------------------------
DROP_BAR_Y = -0.04
DROP_BAR_Z = 0.24
DROP_BAR_LENGTH = 0.64
N_DROP_HOLES = 8


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

    # ---- Hopper shell (widened for drop-box proportion) ----
    spreader.visual(
        mesh_from_geometry(_rect_shell_mesh(), "hopper_shell"),
        material=black_plastic,
        name="hopper_shell",
    )
    spreader.visual(
        _hopper_loop_mesh(0.90, 0.57, 0.845, 0.060, 0.018, "top_rolled_rim"),
        material=black_plastic,
        name="top_rolled_rim",
    )
    spreader.visual(
        _hopper_loop_mesh(0.82, 0.52, 0.615, 0.050, 0.016, "middle_seam_rib"),
        material=black_plastic,
        name="middle_seam_rib",
    )
    spreader.visual(
        _hopper_loop_mesh(0.72, 0.40, 0.400, 0.035, 0.018, "lower_support_ring"),
        material=satin_black,
        name="lower_support_ring",
    )

    # Orange brand-like hopper markings
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

    # ---- Chassis: axle, gearbox ----
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

    # ---- Frame tubes (KEEP) ----
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

    # ---- T-handle group (KEEP) ----
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

    # ---- Distribution chute + gate guide rails (KEEP rails and stop) ----
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
        Box((0.360, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.040, 0.345)),
        material=satin_black,
        name="flow_gate_rear_stop",
    )

    # ---- Drop bar assembly (replaces broadcast spinner) ----
    # Full-width horizontal rectangular metering bar under the hopper throat.
    spreader.visual(
        Box((DROP_BAR_LENGTH, 0.058, 0.058)),
        origin=Origin(xyz=(0.0, DROP_BAR_Y, DROP_BAR_Z)),
        material=satin_black,
        name="drop_bar_body",
    )
    for i, x in enumerate((-0.326, 0.326)):
        spreader.visual(
            Box((0.012, 0.062, 0.062)),
            origin=Origin(xyz=(x, DROP_BAR_Y, DROP_BAR_Z)),
            material=galvanized,
            name=f"drop_bar_endcap_{i}",
        )
    # Vertical brackets weld the drop bar to the axle/frame area.
    for i, x in enumerate((-0.20, 0.20)):
        spreader.visual(
            Box((0.018, 0.018, 0.040)),
            origin=Origin(xyz=(x, DROP_BAR_Y, 0.190)),
            material=satin_black,
            name=f"drop_bar_bracket_{i}",
        )

    # Loop-emitted metering drop holes along the bar bottom.
    drop_hole_span = 0.52
    drop_hole_start = -drop_hole_span / 2.0
    drop_hole_step = drop_hole_span / (N_DROP_HOLES - 1)
    for i in range(N_DROP_HOLES):
        x = drop_hole_start + i * drop_hole_step
        spreader.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, DROP_BAR_Y, DROP_BAR_Z - 0.029)),
            material=dark_hardware,
            name=f"drop_hole_{i}",
        )

    # Agitator drive chain from the gearbox to the drop bar end.
    spreader.visual(
        _tube_mesh(
            [(0.0, -0.02, 0.165), (-0.08, -0.03, 0.178), (-0.18, -0.035, 0.190), (-0.26, DROP_BAR_Y, 0.200)],
            0.005,
            "agitator_drive_chain",
        ),
        material=dark_hardware,
        name="agitator_drive_chain",
    )

    # Control cable from handle control bracket down to the gate area (KEEP).
    spreader.visual(
        _tube_mesh([(0.085, 0.575, 1.055), (0.105, 0.335, 0.820), (0.080, 0.105, 0.560), (0.040, -0.030, 0.382)], 0.0035, "control_cable", samples=10),
        material=galvanized,
        name="control_cable",
    )

    # Axle spacers (KEEP).
    for i, x in enumerate((-0.391, 0.391)):
        spreader.visual(
            Cylinder(radius=0.130, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
            material=galvanized,
            name=f"axle_spacer_{i}",
        )

    # Small dark hardware fasteners (KEEP).
    for idx, (x, y, z) in enumerate(
        [
            (-0.250, -0.180, 0.548),
            (0.250, -0.180, 0.548),
            (-0.045, 0.585, 1.128),
            (0.045, 0.585, 1.128),
        ]
    ):
        spreader.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(x, y, z)),
            material=dark_hardware,
            name=f"fastener_{idx}",
        )

    # ---- Wheels (KEEP) ----
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

    # ---- Agitator (replaces broadcast spinner) ----
    # Rotating rod with paddles running inside the drop bar to prevent seed bridging.
    agitator = model.part("agitator")
    agitator.visual(
        Cylinder(radius=0.010, length=0.52),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="agitator_rod",
    )
    # Alternating paddles make the rotation clearly visible.
    for i in range(6):
        x_paddle = -0.22 + i * 0.088
        z_offset = 0.016 if i % 2 == 0 else -0.016
        agitator.visual(
            Box((0.008, 0.024, 0.016)),
            origin=Origin(xyz=(x_paddle, 0.0, z_offset)),
            material=galvanized,
            name=f"agitator_paddle_{i}",
        )
    # Bearing journals at each rod end.
    for i, x_journal in enumerate((-0.265, 0.265)):
        agitator.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(x_journal, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_hardware,
            name=f"agitator_journal_{i}",
        )
    model.articulation(
        "spreader_to_agitator",
        ArticulationType.CONTINUOUS,
        parent=spreader,
        child=agitator,
        origin=Origin(xyz=(0.0, DROP_BAR_Y, DROP_BAR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=15.0),
    )

    # ---- Control lever (KEEP) ----
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

    # ---- Flow gate (KEEP) ----
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
    agitator = object_model.get_part("agitator")
    gate = object_model.get_part("flow_gate")
    lever = object_model.get_part("control_lever")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    agitator_joint = object_model.get_articulation("spreader_to_agitator")
    lever_joint = object_model.get_articulation("spreader_to_lever")
    gate_joint = object_model.get_articulation("lever_to_gate")

    # --- Wheel bearing captured on axle (KEEP) ---
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

    # --- Agitator rod and journals captured inside drop bar body ---
    ctx.allow_overlap(
        agitator,
        spreader,
        elem_a="agitator_rod",
        elem_b="drop_bar_body",
        reason="The agitator rod is intentionally modeled as running through the drop bar body as a captured shaft.",
    )
    ctx.allow_overlap(
        agitator,
        spreader,
        elem_a="agitator_journal_0",
        elem_b="drop_bar_body",
        reason="The agitator journal bearing is intentionally nested inside the drop bar end region.",
    )
    ctx.allow_overlap(
        agitator,
        spreader,
        elem_a="agitator_journal_1",
        elem_b="drop_bar_body",
        reason="The agitator journal bearing is intentionally nested inside the drop bar end region.",
    )
    # Paddles are intentionally inside the drop bar body (agitator runs through the bar).
    for i in range(6):
        ctx.allow_overlap(
            agitator,
            spreader,
            elem_a=f"agitator_paddle_{i}",
            elem_b="drop_bar_body",
            reason="The agitator paddle is intentionally modeled inside the drop bar body as part of the captured agitator assembly.",
        )
    ctx.expect_within(
        agitator,
        spreader,
        axes="yz",
        inner_elem="agitator_rod",
        outer_elem="drop_bar_body",
        margin=0.005,
        name="agitator rod runs centered inside the drop bar body",
    )
    ctx.expect_overlap(
        agitator,
        spreader,
        axes="x",
        min_overlap=0.50,
        elem_a="agitator_rod",
        elem_b="drop_bar_body",
        name="agitator rod spans the full drop bar width",
    )
    ctx.expect_within(
        agitator,
        spreader,
        axes="yz",
        inner_elem="agitator_paddle_0",
        outer_elem="drop_bar_body",
        margin=0.002,
        name="agitator paddles stay inside the drop bar envelope",
    )

    # --- Gate captured in chute (KEEP) ---
    ctx.allow_overlap(
        gate,
        spreader,
        elem_a="slide_plate",
        elem_b="chute_tray",
        reason="The sliding flow gate is intentionally captured in the simplified chute slot.",
    )

    # --- Classification ---
    ctx.check(
        "classified as seed spreader",
        object_model.name == "agricultural_seed_spreader"
        and object_model.meta.get("category") == "Agricultural"
        and object_model.meta.get("small_class") == "Seed spreader",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # --- Key subassemblies ---
    ctx.check(
        "key drop spreader subassemblies are present",
        all(part is not None for part in (spreader, agitator, gate, lever, wheel_0, wheel_1)),
        details="expected hopper/frame with drop bar, two wheels, agitator, flow gate, and control lever",
    )

    # --- Non-fixed mechanisms (TARGET-specific assertion) ---
    ctx.check(
        "drop bar agitator is a continuous rotation joint",
        agitator_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="agitator must rotate continuously inside the drop bar to prevent seed bridging",
    )
    ctx.check(
        "visible non-fixed mechanisms are authored",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and gate_joint.articulation_type == ArticulationType.PRISMATIC,
        details="lever/gate must move for seed metering control",
    )

    # --- Drop bar metering holes (TARGET multiplicity check) ---
    drop_hole_names = sorted(
        v.name for v in spreader.visuals if v.name is not None and v.name.startswith("drop_hole_")
    )
    ctx.check(
        "drop bar carries 8 loop-emitted metering drop holes",
        len(drop_hole_names) == N_DROP_HOLES
        and drop_hole_names == [f"drop_hole_{i}" for i in range(N_DROP_HOLES)],
        details=f"found {len(drop_hole_names)} drop holes: {drop_hole_names}",
    )

    # --- Distribution chute feeds above agitator in the drop bar ---
    ctx.expect_gap(
        spreader,
        agitator,
        axis="z",
        min_gap=0.005,
        max_gap=0.060,
        positive_elem="chute_tray",
        negative_elem="agitator_rod",
        name="distribution chute outlet sits above the agitator rod",
    )

    # --- Agitator sits under the hopper throat footprint ---
    ctx.expect_overlap(
        agitator,
        spreader,
        axes="x",
        min_overlap=0.40,
        elem_a="agitator_rod",
        elem_b="hopper_shell",
        name="drop bar agitator spans the hopper throat width",
    )

    # --- Gate covers chute inlet area (KEEP) ---
    ctx.expect_overlap(
        gate,
        spreader,
        axes="xy",
        min_overlap=0.030,
        elem_a="slide_plate",
        elem_b="chute_tray",
        name="sliding flow gate covers the chute inlet area",
    )

    # --- Pose checks ---
    rest_gate = ctx.part_world_position(gate)
    rest_knob_aabb = ctx.part_element_world_aabb(lever, elem="lever_knob")
    rest_paddle_aabb = ctx.part_element_world_aabb(agitator, elem="agitator_paddle_0")

    with ctx.pose({lever_joint: 0.60, gate_joint: 0.115, agitator_joint: 1.2}):
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

    # Agitator rotation visible in a separate pose
    with ctx.pose({agitator_joint: pi / 2.0}):
        rotated_paddle_aabb = ctx.part_element_world_aabb(agitator, elem="agitator_paddle_0")

    def _aabb_center_z(aabb):
        return None if aabb is None else (aabb[0][2] + aabb[1][2]) * 0.5

    def _aabb_center_y(aabb):
        return None if aabb is None else (aabb[0][1] + aabb[1][1]) * 0.5

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
    ctx.check(
        "agitator rotation visibly moves paddles",
        rest_paddle_aabb is not None
        and rotated_paddle_aabb is not None
        and (
            abs(_aabb_center_y(rotated_paddle_aabb) - _aabb_center_y(rest_paddle_aabb)) > 0.005
            or abs(_aabb_center_z(rotated_paddle_aabb) - _aabb_center_z(rest_paddle_aabb)) > 0.005
        ),
        details=f"rest_paddle={rest_paddle_aabb}, rotated_paddle={rotated_paddle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
