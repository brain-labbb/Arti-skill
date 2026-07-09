from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    Inertial,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
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
    section_loft,
    tube_from_spline_points,
)


Point3 = tuple[float, float, float]


def _mid(a: Point3, b: Point3) -> Point3:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _dist(a: Point3, b: Point3) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: Point3, b: Point3) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a: Point3, b: Point3, radius: float, material: Material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_dist(a, b)),
        origin=Origin(xyz=_mid(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rounded_rect_loop(width: float, depth: float, z: float, *, corner_radius: float, segments: int = 6) -> list[Point3]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, corner_radius, corner_segments=segments)]


def _hopper_shell_mesh() -> MeshGeometry:
    """Open top, open bottom tapered shell with visible wall thickness."""
    outer_bottom = _rounded_rect_loop(0.40, 0.28, 0.50, corner_radius=0.045, segments=7)
    outer_mid = _rounded_rect_loop(0.74, 0.55, 0.72, corner_radius=0.075, segments=7)
    outer_top = _rounded_rect_loop(1.05, 0.82, 1.08, corner_radius=0.105, segments=7)
    inner_bottom = _rounded_rect_loop(0.24, 0.15, 0.505, corner_radius=0.025, segments=7)
    inner_mid = _rounded_rect_loop(0.64, 0.45, 0.73, corner_radius=0.060, segments=7)
    inner_top = _rounded_rect_loop(0.94, 0.70, 1.04, corner_radius=0.085, segments=7)

    geom = MeshGeometry()

    def add_loop(loop: list[Point3]) -> list[int]:
        return [geom.add_vertex(x, y, z) for x, y, z in loop]

    ob = add_loop(outer_bottom)
    om = add_loop(outer_mid)
    ot = add_loop(outer_top)
    ib = add_loop(inner_bottom)
    im = add_loop(inner_mid)
    it = add_loop(inner_top)

    def connect(loop_a: list[int], loop_b: list[int], reverse: bool = False) -> None:
        n = len(loop_a)
        for i in range(n):
            j = (i + 1) % n
            if reverse:
                geom.add_face(loop_a[i], loop_b[j], loop_b[i])
                geom.add_face(loop_a[i], loop_a[j], loop_b[j])
            else:
                geom.add_face(loop_a[i], loop_b[i], loop_b[j])
                geom.add_face(loop_a[i], loop_b[j], loop_a[j])

    # Exterior and interior wall faces.
    connect(ob, om)
    connect(om, ot)
    connect(ib, im, reverse=True)
    connect(im, it, reverse=True)

    # Rolled-looking top lip and lower throat edge; leave both openings clear.
    n = len(ot)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(ot[i], it[i], it[j])
        geom.add_face(ot[i], it[j], ot[j])
        geom.add_face(ib[i], ob[j], ib[j])
        geom.add_face(ib[i], ob[i], ob[j])
    return geom


def _chute_mesh() -> MeshGeometry:
    rect = _rounded_rect_loop(0.36, 0.22, 0.445, corner_radius=0.025, segments=5)
    circ = [
        (math.cos(2 * math.pi * i / 40) * 0.105, math.sin(2 * math.pi * i / 40) * 0.105, 0.355)
        for i in range(40)
    ]
    # Resample the rectangular loop count to match the circle for the loft.
    rect_40 = [rect[int(i * len(rect) / 40) % len(rect)] for i in range(40)]
    return section_loft([rect_40, circ], cap=False, solid=False)


def _add_spreader_vanes(part, *, count: int, material: Material) -> None:
    """Add N equally-spaced radial broadcast vanes on the spinner disc."""
    for i in range(count):
        angle = i * 2.0 * math.pi / count
        part.visual(
            Box((0.085, 0.012, 0.004)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.105, math.sin(angle) * 0.105, 0.006),
                rpy=(0.0, 0.0, angle + math.radians(12)),
            ),
            material=material,
            name=f"low_radial_spreader_vane_{i}",
        )


def _add_wheel_visuals(part, prefix: str, tire_mat: Material, hub_mat: Material, dark_mat: Material) -> None:
    tire = TireGeometry(
        0.245,
        0.175,
        inner_radius=0.145,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.08),
        tread=TireTread(style="block", depth=0.014, count=22, land_ratio=0.52),
        sidewall=TireSidewall(style="square", bulge=0.035),
        shoulder=TireShoulder(width=0.014, radius=0.004),
    )
    wheel = WheelGeometry(
        0.145,
        0.105,
        rim=WheelRim(inner_radius=0.078, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.004),
        hub=WheelHub(
            radius=0.040,
            width=0.075,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.060, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.018, front_inset=0.010, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.018),
        bore=WheelBore(style="round", diameter=0.020),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=tire_mat, name="treaded_tire")
    part.visual(mesh_from_geometry(wheel, f"{prefix}_silver_hub"), material=hub_mat, name="silver_hub")
    part.visual(Cylinder(radius=0.030, length=0.205), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_mat, name="axle_boss")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tow_behind_seed_spreader",
        meta={
            "category": "Agricultural",
            "small_class": "Seed spreader",
            "description": "Tow-behind seed/fertilizer broadcast spreader with hollow hopper, flow gate, linkage, spinner, red tubular frame, and pneumatic tires.",
        },
    )

    red = model.material("gloss_red_painted_steel", rgba=(0.88, 0.06, 0.02, 1.0))
    olive = model.material("olive_black_poly_hopper", rgba=(0.10, 0.12, 0.055, 1.0))
    rim_black = model.material("black_rolled_poly_rim", rgba=(0.015, 0.017, 0.014, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.018, 0.018, 0.015, 1.0))
    silver = model.material("galvanized_silver_steel", rgba=(0.74, 0.73, 0.68, 1.0))
    dark = model.material("dark_hardware", rgba=(0.05, 0.05, 0.045, 1.0))
    label_red = model.material("red_label_decal", rgba=(1.0, 0.02, 0.01, 1.0))
    label_white = model.material("white_label_swoosh", rgba=(0.95, 0.92, 0.84, 1.0))

    frame = model.part("tow_frame")
    frame.inertial = Inertial.from_geometry(Box((1.60, 2.00, 0.95)), mass=38.0, origin=Origin(xyz=(0.0, -0.05, 0.48)))

    # Main tow tongue, clevis, axle, and triangular red tube frame.
    _add_member(frame, (0.0, -1.35, 0.13), (0.0, -0.28, 0.24), 0.024, red, "front_tongue_tube")
    _add_member(frame, (0.0, -0.42, 0.22), (0.52, 0.44, 0.31), 0.024, red, "side_rail_0")
    _add_member(frame, (0.0, -0.42, 0.22), (-0.52, 0.44, 0.31), 0.024, red, "side_rail_1")
    _add_member(frame, (-0.5575, 0.44, 0.31), (0.5575, 0.44, 0.31), 0.026, silver, "galvanized_axle")
    _add_member(frame, (-0.45, 0.05, 0.29), (0.45, 0.05, 0.29), 0.022, red, "front_cross_tube")
    _add_member(frame, (-0.47, -0.14, 0.490), (0.47, -0.14, 0.490), 0.022, red, "hopper_front_support")
    _add_member(frame, (-0.47, 0.34, 0.480), (0.47, 0.34, 0.480), 0.022, red, "hopper_rear_support")
    _add_member(frame, (-0.47, -0.22, 0.54), (-0.47, 0.43, 0.31), 0.018, red, "diagonal_brace_0")
    _add_member(frame, (0.47, -0.22, 0.54), (0.47, 0.43, 0.31), 0.018, red, "diagonal_brace_1")
    _add_member(frame, (-0.47, -0.22, 0.54), (-0.47, -0.14, 0.49), 0.018, red, "front_support_left_mount")
    _add_member(frame, (0.47, -0.22, 0.54), (0.47, -0.14, 0.49), 0.018, red, "front_support_right_mount")

    for x in (-0.47, 0.47):
        side_loop = tube_from_spline_points(
            [
                (x, 0.43, 0.31),
                (x, 0.34, 0.48),
                (x, 0.16, 0.72),
                (x, -0.10, 0.73),
                (x, -0.22, 0.54),
                (x, -0.10, 0.34),
            ],
            radius=0.025,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        )
        frame.visual(mesh_from_geometry(side_loop, f"red_side_loop_{x:+.0f}"), material=red, name=f"side_loop_{0 if x < 0 else 1}")
        _add_member(frame, (x, -0.10, 0.34), (x * 0.88, 0.36, 0.31), 0.018, red, f"wheel_stay_{0 if x < 0 else 1}")

    # Front height/flow handle mast and hitch clevis.
    front_mast = tube_from_spline_points(
        [
            (0.0, -0.78, 0.12),
            (0.0, -0.78, 0.38),
            (0.0, -0.78, 0.66),
            (0.0, -0.70, 0.82),
        ],
        radius=0.025,
        samples_per_segment=14,
        radial_segments=18,
    )
    frame.visual(mesh_from_geometry(front_mast, "upright_height_handle"), material=red, name="upright_handle")
    frame.visual(Box((0.13, 0.020, 0.28)), origin=Origin(xyz=(0.0, -0.80, 0.58)), material=red, name="lever_quadrant")
    frame.visual(Box((0.18, 0.055, 0.010)), origin=Origin(xyz=(0.0, -1.38, 0.126)), material=red, name="hitch_flat_bar")
    for x in (-0.035, 0.035):
        frame.visual(Box((0.030, 0.18, 0.022)), origin=Origin(xyz=(x, -1.43, 0.116)), material=red, name=f"hitch_clevis_{0 if x < 0 else 1}")
        frame.visual(Cylinder(radius=0.012, length=0.036), origin=Origin(xyz=(x, -1.49, 0.116), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark, name=f"clevis_hole_boss_{0 if x < 0 else 1}")
    # The actual hollow hopper: olive tapered plastic shell with black rolled rim and an open throat.
    hopper = model.part("hopper")
    hopper.inertial = Inertial.from_geometry(Box((1.05, 0.82, 0.60)), mass=10.0, origin=Origin(xyz=(0.0, 0.06, 0.82)))
    hopper.visual(mesh_from_geometry(_hopper_shell_mesh(), "hollow_open_hopper"), material=olive, name="hollow_hopper_shell")
    rim_path = _rounded_rect_loop(1.10, 0.86, 1.085, corner_radius=0.12, segments=7)
    rim_geom = tube_from_spline_points(
        rim_path,
        radius=0.034,
        closed_spline=True,
        samples_per_segment=8,
        radial_segments=18,
        cap_ends=True,
    )
    hopper.visual(mesh_from_geometry(rim_geom, "thick_rolled_top_rim"), material=rim_black, name="rolled_top_rim")
    hopper.visual(mesh_from_geometry(_chute_mesh(), "short_seed_chute"), material=dark, name="seed_chute")
    hopper.visual(Box((0.014, 0.22, 0.065)), origin=Origin(xyz=(-0.18, 0.0, 0.4775)), material=dark, name="left_throat_collar_wall")
    hopper.visual(Box((0.014, 0.22, 0.065)), origin=Origin(xyz=(0.18, 0.0, 0.4775)), material=dark, name="right_throat_collar_wall")
    hopper.visual(Box((0.36, 0.014, 0.065)), origin=Origin(xyz=(0.0, -0.11, 0.4775)), material=dark, name="front_throat_collar_wall")
    # Front logo-like raised red and white marks, built as geometry rather than color-only.
    hopper.visual(Box((0.30, 0.010, 0.045)), origin=Origin(xyz=(0.0, -0.292, 0.855), rpy=(math.radians(-22), 0.0, 0.0)), material=label_red, name="front_brand_bar")
    hopper.visual(Box((0.18, 0.010, 0.018)), origin=Origin(xyz=(0.08, -0.302, 0.895), rpy=(math.radians(-22), 0.0, math.radians(-10))), material=label_white, name="front_brand_swoosh")
    # Sliding gate plate below the throat.  Positive travel uncovers the aperture.
    gate = model.part("flow_gate")
    gate.inertial = Inertial.from_geometry(Box((0.34, 0.24, 0.035)), mass=1.2, origin=Origin())
    gate.visual(Box((0.30, 0.18, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark, name="sliding_gate_plate")

    # Pivoting lever on the front quadrant; its geometry shows the user control for the gate.
    lever = model.part("flow_lever")
    lever.inertial = Inertial.from_geometry(Box((0.16, 0.08, 0.32)), mass=0.5, origin=Origin(xyz=(0.0, 0.0, 0.12)))
    lever.visual(Cylinder(radius=0.016, length=0.17), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=dark, name="lever_pivot_pin")
    lever.visual(Box((0.035, 0.020, 0.29)), origin=Origin(xyz=(0.0, 0.005, 0.145), rpy=(0.0, 0.0, math.radians(-12))), material=red, name="red_flow_lever")
    lever.visual(Box((0.13, 0.055, 0.028)), origin=Origin(xyz=(0.0, -0.012, 0.303)), material=rim_black, name="rubber_lever_grip")
    lever.visual(Cylinder(radius=0.006, length=0.18), origin=Origin(xyz=(0.0, 0.04, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)), material=silver, name="lever_link_stub")

    spinner = model.part("spinner")
    spinner.inertial = Inertial.from_geometry(Cylinder(radius=0.18, length=0.035), mass=1.1, origin=Origin())
    spinner.visual(Cylinder(radius=0.155, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=dark, name="spinner_disc")
    spinner.visual(Cylinder(radius=0.025, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.060)), material=silver, name="vertical_spinner_shaft")
    _add_spreader_vanes(spinner, count=6, material=silver)
    spinner.visual(Cylinder(radius=0.048, length=0.026), origin=Origin(xyz=(0.0, 0.0, -0.010)), material=silver, name="spinner_hub")

    wheel_0 = model.part("wheel_0")
    wheel_0.inertial = Inertial.from_geometry(Cylinder(radius=0.245, length=0.175), mass=7.0, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)))
    _add_wheel_visuals(wheel_0, "wheel_0", rubber, silver, dark)
    wheel_1 = model.part("wheel_1")
    wheel_1.inertial = Inertial.from_geometry(Cylinder(radius=0.245, length=0.175), mass=7.0, origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)))
    _add_wheel_visuals(wheel_1, "wheel_1", rubber, silver, dark)

    model.articulation("frame_to_hopper", ArticulationType.FIXED, parent=frame, child=hopper, origin=Origin())
    model.articulation(
        "hopper_to_gate",
        ArticulationType.PRISMATIC,
        parent=hopper,
        child=gate,
        origin=Origin(xyz=(0.0, 0.045, 0.475)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.115),
    )
    model.articulation(
        "frame_to_lever",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(0.13, -0.812, 0.595)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=math.radians(-28), upper=math.radians(42)),
    )
    model.articulation(
        "hopper_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=hopper,
        child=spinner,
        origin=Origin(xyz=(0.0, 0.06, 0.348)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "axle_to_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.66, 0.44, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "axle_to_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(0.66, 0.44, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("tow_frame")
    hopper = object_model.get_part("hopper")
    gate = object_model.get_part("flow_gate")
    spinner = object_model.get_part("spinner")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    gate_joint = object_model.get_articulation("hopper_to_gate")
    lever_joint = object_model.get_articulation("frame_to_lever")
    spinner_joint = object_model.get_articulation("hopper_to_spinner")
    wheel_joint = object_model.get_articulation("axle_to_wheel_0")

    ctx.allow_overlap(
        hopper,
        spinner,
        elem_a="seed_chute",
        elem_b="vertical_spinner_shaft",
        reason="The spinner drive shaft is intentionally shown passing up through the short seed chute below the hopper throat.",
    )
    ctx.allow_overlap(
        "flow_lever",
        "tow_frame",
        elem_a="lever_pivot_pin",
        elem_b="lever_quadrant",
        reason="The lever pivot pin is intentionally captured through the slotted red control quadrant.",
    )
    ctx.allow_overlap(
        "tow_frame",
        "hopper",
        elem_a="hopper_front_support",
        elem_b="hollow_hopper_shell",
        reason="The red tubular saddle is intentionally tucked into the plastic hopper's lower mounting lip to show a bolted cradle support.",
    )

    ctx.check(
        "asset is named as Agricultural Seed spreader",
        object_model.name == "tow_behind_seed_spreader"
        and object_model.meta.get("category") == "Agricultural"
        and object_model.meta.get("small_class") == "Seed spreader",
        details=f"name={object_model.name} meta={object_model.meta}",
    )
    ctx.check(
        "primary seed spreader mechanisms exist",
        gate_joint.articulation_type == ArticulationType.PRISMATIC
        and lever_joint.articulation_type == ArticulationType.REVOLUTE
        and spinner_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="expected prismatic gate, revolute lever, continuous spinner and wheel",
    )
    ctx.expect_overlap(hopper, gate, axes="xy", elem_a="hollow_hopper_shell", elem_b="sliding_gate_plate", min_overlap=0.16, name="gate sits directly under hopper throat")
    ctx.expect_gap(gate, spinner, axis="z", min_gap=0.055, max_gap=0.16, positive_elem="sliding_gate_plate", negative_elem="spinner_disc", name="chute has clear space to spinner")
    ctx.expect_origin_distance(wheel_0, wheel_1, axes="x", min_dist=1.20, max_dist=1.40, name="large wheels straddle frame")
    ctx.expect_overlap(frame, hopper, axes="x", elem_a="hopper_front_support", elem_b="hollow_hopper_shell", min_overlap=0.70, name="red frame support spans hopper width")
    ctx.expect_overlap(spinner, hopper, axes="xy", elem_a="spinner_disc", elem_b="seed_chute", min_overlap=0.12, name="spinner centered below seed chute")

    # Variant axis: 6 equally-spaced broadcast vanes fixed to the spinner hub.
    vane_names = [f"low_radial_spreader_vane_{i}" for i in range(6)]
    spinner_visual_names = {v.name for v in spinner.visuals}
    ctx.check(
        "spinner carries 6 broadcast vanes (fork variant N=6)",
        all(vn in spinner_visual_names for vn in vane_names),
        details=f"expected={vane_names}, found={sorted(spinner_visual_names)}",
    )
    ctx.expect_overlap("flow_lever", "tow_frame", axes="yz", elem_a="lever_pivot_pin", elem_b="lever_quadrant", min_overlap=0.014, name="lever pivot is retained by quadrant")

    rest_gate = ctx.part_world_position(gate)
    with ctx.pose({gate_joint: 0.115, lever_joint: math.radians(35), spinner_joint: 1.2, wheel_joint: math.pi / 2.0}):
        open_gate = ctx.part_world_position(gate)
        ctx.expect_overlap(hopper, gate, axes="x", elem_a="hollow_hopper_shell", elem_b="sliding_gate_plate", min_overlap=0.20, name="opened gate remains retained side-to-side")
        ctx.expect_gap(gate, spinner, axis="z", min_gap=0.055, max_gap=0.17, positive_elem="sliding_gate_plate", negative_elem="spinner_disc", name="opened gate still feeds down to spinner")
    ctx.check(
        "positive gate travel visibly opens rearward",
        rest_gate is not None and open_gate is not None and open_gate[1] > rest_gate[1] + 0.09,
        details=f"rest_gate={rest_gate}, open_gate={open_gate}",
    )

    return ctx.report()


object_model = build_object_model()
