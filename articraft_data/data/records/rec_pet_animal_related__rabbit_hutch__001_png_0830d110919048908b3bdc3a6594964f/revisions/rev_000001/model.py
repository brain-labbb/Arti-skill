from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WOOD = Material("warm white painted wood", rgba=(0.86, 0.84, 0.78, 1.0))
TRIM = Material("weathered grey trim", rgba=(0.55, 0.56, 0.54, 1.0))
MESH = Material("dark galvanized wire mesh", rgba=(0.10, 0.11, 0.10, 1.0))
BLACK = Material("black latch hardware", rgba=(0.02, 0.02, 0.018, 1.0))
TRAY = Material("black removable plastic tray", rgba=(0.035, 0.035, 0.032, 1.0))
CLEAR = Material("slightly smoky clear acrylic", rgba=(0.62, 0.72, 0.78, 0.38))
RAMP = Material("pale grooved ramp planks", rgba=(0.72, 0.67, 0.58, 1.0))


def _box(part, name: str, size, xyz, material=WOOD, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material=BLACK, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _front_mesh(part, prefix: str, x0: float, x1: float, z0: float, z1: float, y: float, *, wire=0.006, depth=0.010) -> None:
    """Square wire grid in an XZ panel at fixed Y."""
    width = x1 - x0
    height = z1 - z0
    cols = max(3, int(width / 0.095))
    rows = max(3, int(height / 0.085))
    for i in range(cols + 1):
        x = x0 + width * i / cols
        _box(part, f"{prefix}_v_{i}", (wire, depth, height + wire), (x, y, (z0 + z1) / 2), MESH)
    for j in range(rows + 1):
        z = z0 + height * j / rows
        _box(part, f"{prefix}_h_{j}", (width + wire, depth, wire), ((x0 + x1) / 2, y, z), MESH)


def _side_mesh(part, prefix: str, y0: float, y1: float, z0: float, z1: float, x: float, *, wire=0.006, depth=0.010) -> None:
    """Square wire grid in a YZ panel at fixed X."""
    panel_depth = y1 - y0
    height = z1 - z0
    cols = max(3, int(panel_depth / 0.095))
    rows = max(3, int(height / 0.085))
    for i in range(cols + 1):
        y = y0 + panel_depth * i / cols
        _box(part, f"{prefix}_v_{i}", (depth, wire, height + wire), (x, y, (z0 + z1) / 2), MESH)
    for j in range(rows + 1):
        z = z0 + height * j / rows
        _box(part, f"{prefix}_h_{j}", (depth, panel_depth + wire, wire), (x, (y0 + y1) / 2, z), MESH)


def _door_frame(part, prefix: str, width: float, height: float, *, mesh: bool, acrylic: bool = False) -> None:
    """Door local frame: hinge line at local X=0, door extends along -X, vertical Z."""
    stile = 0.038
    thick = 0.030
    _box(part, f"{prefix}_hinge_stile", (stile, thick, height), (-stile / 2, 0.0, 0.0), WOOD)
    _box(part, f"{prefix}_free_stile", (stile, thick, height), (-width + stile / 2, 0.0, 0.0), WOOD)
    _box(part, f"{prefix}_top_rail", (width, thick, stile), (-width / 2, 0.0, height / 2 - stile / 2), WOOD)
    _box(part, f"{prefix}_bottom_rail", (width, thick, stile), (-width / 2, 0.0, -height / 2 + stile / 2), WOOD)
    _box(part, f"{prefix}_inner_lip", (width - 0.075, 0.010, height - 0.075), (-width / 2, -0.003, 0.0), TRIM)
    if acrylic:
        _box(part, f"{prefix}_clear_window", (width - 0.092, 0.006, height - 0.100), (-width / 2, -0.006, 0.0), CLEAR)
    if mesh:
        _front_mesh(
            part,
            f"{prefix}_mesh",
            -width + 0.052,
            -0.052,
            -height / 2 + 0.052,
            height / 2 - 0.052,
            -0.008,
            wire=0.0045,
            depth=0.006,
        )
    # Two exposed hinge knuckles and a small latch plate are attached to the door.
    _cyl(part, f"{prefix}_hinge_pin_upper", 0.010, 0.085, (0.0, -0.002, height * 0.25), BLACK)
    _cyl(part, f"{prefix}_hinge_pin_lower", 0.010, 0.085, (0.0, -0.002, -height * 0.25), BLACK)
    _box(part, f"{prefix}_hinge_leaf_upper", (0.050, 0.008, 0.055), (-0.028, -0.017, height * 0.25), BLACK)
    _box(part, f"{prefix}_hinge_leaf_lower", (0.050, 0.008, 0.055), (-0.028, -0.017, -height * 0.25), BLACK)
    _box(part, f"{prefix}_latch_plate", (0.042, 0.014, 0.026), (-width + 0.032, -0.022, 0.0), BLACK)
    _cyl(part, f"{prefix}_latch_knob", 0.010, 0.016, (-width + 0.010, -0.035, 0.0), BLACK, rpy=(math.pi / 2, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_compartment_rabbit_hutch")
    for mat in (WOOD, TRIM, MESH, BLACK, TRAY, CLEAR, RAMP):
        model.material(mat.name, rgba=mat.rgba)

    frame = model.part("hutch_frame")

    # Real-world footprint: a compact wheeled hutch/run, about 1.65 m wide.
    x_left, x_mid, x_right = -0.80, -0.18, 0.84
    y_front, y_back = -0.36, 0.36
    z_leg_bottom, z_floor, z_mid, z_top = 0.10, 0.30, 0.70, 1.08
    post = 0.045

    # Corner and divider posts carry the body and define the two-compartment layout.
    for ix, x in enumerate((x_left, x_mid, x_right)):
        for iy, y in enumerate((y_front, y_back)):
            _box(frame, f"post_{ix}_{iy}", (post, post, z_top - z_leg_bottom), (x, y, (z_top + z_leg_bottom) / 2))

    # Front/back rails and side rails at bottom, mid-shelf, and top.
    for iz, z in enumerate((z_floor, z_mid, z_top)):
        _box(frame, f"front_rail_{iz}", (x_right - x_left + post, post, post), ((x_left + x_right) / 2, y_front, z))
        _box(frame, f"back_rail_{iz}", (x_right - x_left + post, post, post), ((x_left + x_right) / 2, y_back, z))
        _box(frame, f"left_side_rail_{iz}", (post, y_back - y_front + post, post), (x_left, 0.0, z))
        _box(frame, f"divider_side_rail_{iz}", (post, y_back - y_front + post, post), (x_mid, 0.0, z))
        _box(frame, f"right_side_rail_{iz}", (post, y_back - y_front + post, post), (x_right, 0.0, z))

    # Additional shelf and platform structure visible in the reference hutch.
    _box(frame, "left_mid_shelf", (0.63, 0.72, 0.026), (-0.49, 0.0, z_mid - 0.020), WOOD)
    _box(frame, "run_upper_platform", (0.34, 0.62, 0.030), (0.00, 0.0, 0.745), WOOD)
    _box(frame, "run_back_floor_rail", (0.98, 0.130, 0.030), (0.33, 0.320, z_floor + 0.005), TRIM)
    _box(frame, "run_front_floor_rail", (0.98, 0.130, 0.030), (0.33, -0.320, z_floor + 0.005), TRIM)

    # Solid/planked left hutch back and side walls.
    _box(frame, "upper_back_panel", (0.56, 0.018, 0.34), (-0.49, y_back + 0.018, 0.885), WOOD)
    for i, z in enumerate((0.790, 0.875, 0.960)):
        _box(frame, f"upper_back_plank_line_{i}", (0.54, 0.010, 0.008), (-0.49, y_back + 0.030, z), TRIM)
    _box(frame, "left_solid_side", (0.018, 0.64, 0.36), (x_left - 0.010, 0.0, 0.880), WOOD)
    _box(frame, "left_lower_back_mesh_frame", (0.62, 0.030, 0.32), (-0.49, y_back + 0.005, 0.500), TRIM)

    # Wire mesh panels on the run sides and back preserve the large square openings.
    _front_mesh(frame, "run_back_mesh", x_mid + 0.055, x_right - 0.055, z_floor + 0.055, z_top - 0.065, y_back + 0.010)
    _side_mesh(frame, "run_side_mesh", y_front + 0.055, y_back - 0.055, z_floor + 0.055, z_top - 0.065, x_right + 0.010)
    _side_mesh(frame, "divider_mesh", y_front + 0.055, y_back - 0.055, z_floor + 0.055, z_top - 0.065, x_mid + 0.010)

    # Fixed roof over the enclosed hutch with slight overhang and grey weather cap.
    _box(frame, "left_roof_overhang", (0.70, 0.82, 0.035), (-0.50, 0.0, 1.115), TRIM)
    _box(frame, "left_roof_white_edge", (0.66, 0.76, 0.018), (-0.50, 0.0, 1.090), WOOD)
    _box(frame, "front_roof_lip", (1.68, 0.038, 0.035), (0.02, y_front - 0.025, 1.070), WOOD)
    _box(frame, "back_roof_lip", (1.68, 0.038, 0.035), (0.02, y_back + 0.025, 1.070), WOOD)

    # Internal diagonal ramp from the run floor to the upper platform, with cleats.
    ramp_angle = math.atan2(0.38, 0.60)
    _box(frame, "internal_ramp_board", (0.78, 0.18, 0.034), (0.32, 0.03, 0.605), RAMP, rpy=(0.0, -ramp_angle, 0.0))
    for i in range(5):
        t = -0.25 + i * 0.125
        x = 0.32 + t * math.cos(ramp_angle)
        z = 0.605 + t * math.sin(ramp_angle)
        _box(frame, f"internal_ramp_cleat_{i}", (0.020, 0.19, 0.018), (x, 0.03, z + 0.020), TRIM, rpy=(0.0, -ramp_angle, 0.0))
    _box(frame, "internal_ramp_lower_bracket", (0.060, 0.070, 0.055), (0.005, 0.330, 0.390), WOOD)
    _box(frame, "internal_ramp_upper_bracket", (0.060, 0.070, 0.055), (0.625, 0.330, 0.790), WOOD)
    _box(frame, "internal_ramp_lower_ledger", (0.060, 0.340, 0.050), (0.005, 0.180, 0.410), WOOD)
    _box(frame, "internal_ramp_upper_ledger", (0.060, 0.340, 0.050), (0.625, 0.180, 0.770), WOOD)

    # Caster forks/plates are fixed to the legs; wheels are separate articulated parts.
    wheel_centers = [
        (x_left + 0.035, y_front + 0.035, 0.055),
        (x_left + 0.035, y_back - 0.035, 0.055),
        (x_right - 0.035, y_front + 0.035, 0.055),
        (x_right - 0.035, y_back - 0.035, 0.055),
    ]
    for i, (x, y, z) in enumerate(wheel_centers):
        _box(frame, f"caster_plate_{i}", (0.080, 0.070, 0.010), (x, y, 0.102), BLACK)
        _box(frame, f"caster_fork_a_{i}", (0.006, 0.055, 0.070), (x - 0.016, y, 0.067), BLACK)
        _box(frame, f"caster_fork_b_{i}", (0.006, 0.055, 0.070), (x + 0.016, y, 0.067), BLACK)

    # Fixed hinge receiver plates protrude from the posts to meet the moving hinge knuckles.
    for zc, h in ((0.875, 0.30), (0.505, 0.30)):
        for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
            pin_z = zc + sign * h * 0.25
            _box(frame, f"{suffix}_left_hinge_receiver_{int(pin_z * 1000)}", (0.014, 0.020, 0.055), (x_mid - 0.021, y_front - 0.032, pin_z), BLACK)
    for sign, suffix in ((1.0, "top"), (-1.0, "bottom")):
        pin_z = 0.690 + sign * 0.62 * 0.25
        _box(frame, f"run_{suffix}_hinge_receiver", (0.014, 0.020, 0.065), (x_right - 0.023, y_front - 0.032, pin_z), BLACK)
    _box(frame, "front_ramp_hinge_receiver", (0.40, 0.034, 0.030), (-0.50, y_front - 0.033, z_floor - 0.010), BLACK)

    # Movable upper acrylic door on the left compartment.
    upper_door = model.part("upper_door")
    _door_frame(upper_door, "upper_door", width=0.50, height=0.30, mesh=False, acrylic=True)
    model.articulation(
        "frame_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_door,
        origin=Origin(xyz=(x_mid - 0.028, y_front - 0.050, 0.875)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=6.0, velocity=1.5),
    )

    # Lower front mesh door.
    lower_door = model.part("lower_door")
    _door_frame(lower_door, "lower_door", width=0.50, height=0.30, mesh=True, acrylic=False)
    model.articulation(
        "frame_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_door,
        origin=Origin(xyz=(x_mid - 0.028, y_front - 0.050, 0.505)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=6.0, velocity=1.5),
    )

    # Large run access door on the right, framed wire mesh like the image.
    run_door = model.part("run_door")
    _door_frame(run_door, "run_door", width=0.88, height=0.62, mesh=True, acrylic=False)
    model.articulation(
        "frame_to_run_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=run_door,
        origin=Origin(xyz=(x_right - 0.030, y_front - 0.052, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=7.0, velocity=1.2),
    )

    # Hinged top access roof over the right run.
    roof_lid = model.part("roof_lid")
    _box(roof_lid, "roof_panel", (0.95, 0.78, 0.035), (0.0, -0.39, 0.018), TRIM)
    _box(roof_lid, "roof_front_edge", (0.95, 0.035, 0.040), (0.0, -0.775, -0.010), WOOD)
    _box(roof_lid, "roof_side_edge_a", (0.035, 0.78, 0.038), (-0.475, -0.39, -0.012), WOOD)
    _box(roof_lid, "roof_side_edge_b", (0.035, 0.78, 0.038), (0.475, -0.39, -0.012), WOOD)
    _cyl(roof_lid, "roof_piano_hinge", 0.009, 0.90, (0.0, 0.0, -0.006), BLACK, rpy=(0.0, math.pi / 2, 0.0))
    model.articulation(
        "frame_to_roof_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=roof_lid,
        origin=Origin(xyz=(0.35, y_back + 0.030, 1.1335)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=8.0, velocity=1.0),
    )

    # Front drop ramp, shown lowered from the left lower compartment.
    front_ramp = model.part("front_ramp")
    down_angle = math.radians(24.0)
    center_y = -0.28 * math.cos(down_angle)
    center_z = -0.28 * math.sin(down_angle)
    _box(front_ramp, "ramp_board", (0.34, 0.56, 0.026), (0.0, center_y, center_z), RAMP, rpy=(down_angle, 0.0, 0.0))
    _box(front_ramp, "ramp_side_rail_a", (0.030, 0.45, 0.035), (-0.185, center_y - 0.055, center_z - 0.025), WOOD, rpy=(down_angle, 0.0, 0.0))
    _box(front_ramp, "ramp_side_rail_b", (0.030, 0.45, 0.035), (0.185, center_y - 0.055, center_z - 0.025), WOOD, rpy=(down_angle, 0.0, 0.0))
    for j in range(4):
        y = -0.10 - j * 0.10
        z = y * math.sin(down_angle) + 0.022
        _box(front_ramp, f"ramp_cleat_{j}", (0.30, 0.022, 0.018), (0.0, y * math.cos(down_angle), z), TRIM, rpy=(down_angle, 0.0, 0.0))
    _cyl(front_ramp, "ramp_hinge_rod", 0.008, 0.36, (0.0, 0.0, 0.0), BLACK, rpy=(0.0, math.pi / 2, 0.0))
    model.articulation(
        "frame_to_front_ramp",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_ramp,
        origin=Origin(xyz=(-0.50, y_front - 0.055, z_floor - 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=5.0, velocity=1.0),
    )

    # Removable black droppings tray below the right run, on rails, sliding out the front.
    tray = model.part("floor_tray")
    _box(tray, "tray_pan", (0.86, 0.58, 0.025), (0.0, 0.0, 0.0), TRAY)
    _box(tray, "tray_front_pull", (0.28, 0.018, 0.030), (0.0, -0.293, 0.018), BLACK)
    model.articulation(
        "frame_to_floor_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.32, 0.0, z_floor + 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.34, effort=25.0, velocity=0.25),
    )

    # Four rolling casters; cylindrical wheels rotate around their horizontal axles.
    for i, (x, y, z) in enumerate(wheel_centers):
        wheel = model.part(f"caster_wheel_{i}")
        _cyl(wheel, f"wheel_tire_{i}", 0.034, 0.026, (0.0, 0.0, 0.0), BLACK, rpy=(0.0, math.pi / 2, 0.0))
        _cyl(wheel, f"wheel_hub_{i}", 0.014, 0.030, (0.0, 0.0, 0.0), TRIM, rpy=(0.0, math.pi / 2, 0.0))
        model.articulation(
            f"fork_to_caster_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("hutch_frame")
    upper = object_model.get_part("upper_door")
    lower = object_model.get_part("lower_door")
    run = object_model.get_part("run_door")
    roof = object_model.get_part("roof_lid")
    ramp = object_model.get_part("front_ramp")
    tray = object_model.get_part("floor_tray")

    upper_hinge = object_model.get_articulation("frame_to_upper_door")
    run_hinge = object_model.get_articulation("frame_to_run_door")
    roof_hinge = object_model.get_articulation("frame_to_roof_lid")
    ramp_hinge = object_model.get_articulation("frame_to_front_ramp")
    tray_slide = object_model.get_articulation("frame_to_floor_tray")

    ctx.expect_overlap(upper, frame, axes="xz", min_overlap=0.20, name="upper door sits in left upper bay")
    ctx.expect_overlap(lower, frame, axes="xz", min_overlap=0.20, name="lower door sits in left lower bay")
    ctx.expect_overlap(run, frame, axes="xz", min_overlap=0.45, name="run door covers the large mesh bay")
    ctx.expect_gap(
        roof,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.060,
        positive_elem="roof_panel",
        negative_elem="front_rail_2",
        name="roof lid rests just above run top frame",
    )
    ctx.expect_within(tray, frame, axes="x", margin=0.02, name="tray remains between run side posts")
    ctx.expect_overlap(tray, frame, axes="y", min_overlap=0.45, name="tray is inserted on its rails")

    upper_closed = ctx.part_world_aabb(upper)
    run_closed = ctx.part_world_aabb(run)
    roof_closed = ctx.part_world_aabb(roof)
    ramp_down = ctx.part_world_aabb(ramp)
    tray_home = ctx.part_world_aabb(tray)
    with ctx.pose({upper_hinge: 1.1, run_hinge: 1.0, roof_hinge: 0.75, ramp_hinge: 1.0, tray_slide: 0.25}):
        upper_open = ctx.part_world_aabb(upper)
        run_open = ctx.part_world_aabb(run)
        roof_open = ctx.part_world_aabb(roof)
        ramp_up = ctx.part_world_aabb(ramp)
        tray_out = ctx.part_world_aabb(tray)
        ctx.expect_overlap(tray, frame, axes="x", min_overlap=0.60, name="extended tray still spans the run rails")

    ctx.check(
        "front doors open outward",
        upper_closed is not None
        and upper_open is not None
        and run_closed is not None
        and run_open is not None
        and upper_open[0][1] < upper_closed[0][1] - 0.05
        and run_open[0][1] < run_closed[0][1] - 0.05,
        details=f"upper {upper_closed}->{upper_open}, run {run_closed}->{run_open}",
    )
    ctx.check(
        "roof lid opens upward",
        roof_closed is not None and roof_open is not None and roof_open[1][2] > roof_closed[1][2] + 0.10,
        details=f"roof {roof_closed}->{roof_open}",
    )
    ctx.check(
        "front ramp folds upward",
        ramp_down is not None and ramp_up is not None and ramp_up[1][2] > ramp_down[1][2] + 0.12,
        details=f"ramp {ramp_down}->{ramp_up}",
    )
    ctx.check(
        "tray slides out toward front",
        tray_home is not None and tray_out is not None and tray_out[0][1] < tray_home[0][1] - 0.20,
        details=f"tray {tray_home}->{tray_out}",
    )

    return ctx.report()


object_model = build_object_model()
