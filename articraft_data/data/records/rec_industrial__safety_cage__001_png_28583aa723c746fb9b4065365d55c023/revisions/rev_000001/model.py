from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 1.60
DEPTH = 1.20
HEIGHT = 2.20
TUBE = 0.060
ROD = 0.010
MESH_FRAME_BITE = 0.085


def _box(part, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material: str):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_lattice_panel(
    part,
    *,
    prefix: str,
    plane: str,
    fixed: float,
    span_a: tuple[float, float],
    span_z: tuple[float, float],
    material: str,
    vertical_count: int,
    horizontal_count: int,
):
    """Add a connected welded-wire grid on an XZ or YZ panel."""
    z0, z1 = span_z
    a0, a1 = span_a
    zc = (z0 + z1) / 2.0
    ac = (a0 + a1) / 2.0
    h = z1 - z0
    a_len = a1 - a0
    z_rod_len = h + 2.0 * MESH_FRAME_BITE
    a_rod_len = a_len + 2.0 * MESH_FRAME_BITE

    for i in range(vertical_count):
        t = i / max(1, vertical_count - 1)
        a = a0 + (a1 - a0) * t
        if plane == "xz":
            _box(part, f"{prefix}_v_{i}", (ROD, ROD, z_rod_len), (a, fixed, zc), material)
        else:
            _box(part, f"{prefix}_v_{i}", (ROD, ROD, z_rod_len), (fixed, a, zc), material)

    for j in range(horizontal_count):
        t = j / max(1, horizontal_count - 1)
        z = z0 + (z1 - z0) * t
        if plane == "xz":
            _box(part, f"{prefix}_h_{j}", (a_rod_len, ROD, ROD), (ac, fixed, z), material)
        else:
            _box(part, f"{prefix}_h_{j}", (ROD, a_rod_len, ROD), (fixed, ac, z), material)


def _add_left_door_leaf(part, *, material: str, mesh_material: str):
    """Closed left leaf: hinge line at local X=0, the leaf extends along +X."""
    door_w = 0.72
    door_h = 1.92
    z0 = 0.12
    zc = z0 + door_h / 2.0
    tube = 0.052

    _box(part, "hinge_stile", (tube, tube, door_h), (0.0, 0.0, zc), material)
    _box(part, "latch_stile", (tube, tube, door_h), (door_w, 0.0, zc), material)
    _box(part, "top_rail", (door_w + tube, tube, tube), (door_w / 2.0, 0.0, z0 + door_h), material)
    _box(part, "bottom_rail", (door_w + tube, tube, tube), (door_w / 2.0, 0.0, z0), material)
    _box(part, "middle_rail", (door_w + tube, tube, tube), (door_w / 2.0, 0.0, 1.08), material)
    _box(part, "diagonal_brace", (0.050, tube, 1.22), (0.36, 0.0, 0.82), material)

    for i, x in enumerate([0.10, 0.20, 0.30, 0.42, 0.52, 0.62]):
        _box(part, f"door_mesh_v_{i}", (ROD, ROD, door_h), (x, 0.0, zc), mesh_material)
    for j, z in enumerate([0.24, 0.40, 0.56, 0.72, 0.88, 1.24, 1.40, 1.56, 1.72, 1.88]):
        _box(part, f"door_mesh_h_{j}", (door_w + 0.020, ROD, ROD), (door_w / 2.0, 0.0, z), mesh_material)

    for k, z in enumerate([0.42, 1.10, 1.76]):
        _cylinder(part, f"hinge_knuckle_{k}", 0.026, 0.24, (0.0, 0.0, z), material)
    _box(part, "latch_tongue", (0.070, 0.026, 0.14), (door_w + 0.012, -0.032, 1.07), "black_hardware")


def _add_right_door_leaf(part, *, material: str, mesh_material: str):
    """Open right leaf: hinge line at local Y=0, the leaf extends outward along -Y."""
    door_w = 0.72
    door_h = 1.92
    z0 = 0.12
    zc = z0 + door_h / 2.0
    tube = 0.052

    _box(part, "hinge_stile", (tube, tube, door_h), (0.0, 0.0, zc), material)
    _box(part, "latch_stile", (tube, tube, door_h), (0.0, -door_w, zc), material)
    _box(part, "top_rail", (tube, door_w + tube, tube), (0.0, -door_w / 2.0, z0 + door_h), material)
    _box(part, "bottom_rail", (tube, door_w + tube, tube), (0.0, -door_w / 2.0, z0), material)
    _box(part, "middle_rail", (tube, door_w + tube, tube), (0.0, -door_w / 2.0, 1.08), material)
    _box(part, "diagonal_brace", (tube, 0.050, 1.22), (0.0, -0.36, 0.82), material)

    for i, y in enumerate([-0.10, -0.20, -0.30, -0.42, -0.52, -0.62]):
        _box(part, f"door_mesh_v_{i}", (ROD, ROD, door_h), (0.0, y, zc), mesh_material)
    for j, z in enumerate([0.24, 0.40, 0.56, 0.72, 0.88, 1.24, 1.40, 1.56, 1.72, 1.88]):
        _box(part, f"door_mesh_h_{j}", (ROD, door_w + 0.020, ROD), (0.0, -door_w / 2.0, z), mesh_material)

    for k, z in enumerate([0.42, 1.10, 1.76]):
        _cylinder(part, f"hinge_knuckle_{k}", 0.026, 0.24, (0.0, 0.0, z), material)
    _box(part, "latch_plate", (0.014, 0.10, 0.24), (-0.033, -0.680, 1.10), "black_hardware")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="industrial_safety_cage",
        meta={
            "description": "Yellow welded industrial machine safety cage with wire-mesh panels, hinged front access leaves, latch hardware, base plates, roof grid, and protective rails."
        },
    )
    model.material("safety_yellow", rgba=(1.0, 0.78, 0.02, 1.0))
    model.material("yellow_wire_mesh", rgba=(1.0, 0.86, 0.10, 0.70))
    model.material("black_hardware", rgba=(0.02, 0.02, 0.018, 1.0))
    model.material("galvanized_pin", rgba=(0.55, 0.56, 0.54, 1.0))

    cage = model.part("safety_cage")

    xh = WIDTH / 2.0
    yh = DEPTH / 2.0

    # Base plates under all major uprights.
    for i, x in enumerate([-xh, xh]):
        for j, y in enumerate([-yh, yh]):
            _box(cage, f"base_plate_{i}_{j}", (0.18, 0.18, 0.014), (x, y, 0.007), "safety_yellow")
    for i, x in enumerate([-0.03, 0.03]):
        _box(cage, f"center_base_plate_{i}", (0.16, 0.16, 0.014), (x, -yh, 0.007), "safety_yellow")

    # Vertical square-tube posts.
    for i, x in enumerate([-xh, xh]):
        for j, y in enumerate([-yh, yh]):
            _box(cage, f"corner_post_{i}_{j}", (TUBE, TUBE, HEIGHT), (x, y, HEIGHT / 2.0), "safety_yellow")
    for name, x, y in [
        ("front_mullion", 0.0, -yh),
        ("rear_mullion", 0.0, yh),
        ("side_post_0", -xh, 0.0),
        ("side_post_1", xh, 0.0),
    ]:
        _box(cage, name, (TUBE, TUBE, HEIGHT), (x, y, HEIGHT / 2.0), "safety_yellow")

    # Perimeter rails at base, mid-height, and roof.
    for level_name, z in [("base", 0.055), ("mid", 1.08), ("top", HEIGHT - 0.030)]:
        _box(cage, f"{level_name}_front_rail", (WIDTH + TUBE, TUBE, TUBE), (0.0, -yh, z), "safety_yellow")
        _box(cage, f"{level_name}_rear_rail", (WIDTH + TUBE, TUBE, TUBE), (0.0, yh, z), "safety_yellow")
        _box(cage, f"{level_name}_side_rail_0", (TUBE, DEPTH + TUBE, TUBE), (-xh, 0.0, z), "safety_yellow")
        _box(cage, f"{level_name}_side_rail_1", (TUBE, DEPTH + TUBE, TUBE), (xh, 0.0, z), "safety_yellow")

    # Door header and fixed latch receiver on the central front post.
    _box(cage, "door_header", (WIDTH + TUBE, 0.080, 0.070), (0.0, -yh - 0.005, 2.12), "safety_yellow")
    _box(cage, "latch_receiver", (0.040, 0.018, 0.28), (0.0, -yh - 0.039, 1.10), "black_hardware")

    # Side, rear, roof, and floor welded-wire panels.
    _add_lattice_panel(
        cage,
        prefix="rear_mesh",
        plane="xz",
        fixed=yh + 0.004,
        span_a=(-0.72, 0.72),
        span_z=(0.12, 2.06),
        material="yellow_wire_mesh",
        vertical_count=11,
        horizontal_count=14,
    )
    _add_lattice_panel(
        cage,
        prefix="side_mesh_0",
        plane="yz",
        fixed=-xh - 0.004,
        span_a=(-0.52, 0.52),
        span_z=(0.12, 2.06),
        material="yellow_wire_mesh",
        vertical_count=9,
        horizontal_count=14,
    )
    _add_lattice_panel(
        cage,
        prefix="side_mesh_1",
        plane="yz",
        fixed=xh + 0.004,
        span_a=(-0.52, 0.52),
        span_z=(0.12, 2.06),
        material="yellow_wire_mesh",
        vertical_count=9,
        horizontal_count=14,
    )

    for i, x in enumerate([-0.60, -0.40, -0.20, 0.0, 0.20, 0.40, 0.60]):
        _box(cage, f"roof_mesh_x_{i}", (ROD, DEPTH + TUBE, ROD), (x, 0.0, HEIGHT + 0.005), "yellow_wire_mesh")
        _box(cage, f"floor_mesh_x_{i}", (ROD, DEPTH + TUBE, ROD), (x, 0.0, 0.070), "yellow_wire_mesh")
    for j, y in enumerate([-0.42, -0.24, -0.06, 0.12, 0.30, 0.48]):
        _box(cage, f"roof_mesh_y_{j}", (WIDTH + TUBE, ROD, ROD), (0.0, y, HEIGHT + 0.005), "yellow_wire_mesh")
        _box(cage, f"floor_mesh_y_{j}", (WIDTH + TUBE, ROD, ROD), (0.0, y, 0.070), "yellow_wire_mesh")

    # Small lift loops on the roof corners, like the reference cage.
    for i, x in enumerate([-0.66, 0.66]):
        for j, y in enumerate([-0.57, 0.57]):
            _cylinder(cage, f"lift_loop_leg_a_{i}_{j}", 0.008, 0.11, (x - 0.025, y, HEIGHT + 0.045), "safety_yellow")
            _cylinder(cage, f"lift_loop_leg_b_{i}_{j}", 0.008, 0.11, (x + 0.025, y, HEIGHT + 0.045), "safety_yellow")
            _cylinder(
                cage,
                f"lift_loop_top_{i}_{j}",
                0.008,
                0.050,
                (x, y, HEIGHT + 0.095),
                "safety_yellow",
                rpy=(0.0, math.pi / 2.0, 0.0),
            )

    left_door = model.part("left_door")
    _add_left_door_leaf(left_door, material="safety_yellow", mesh_material="yellow_wire_mesh")

    right_door = model.part("right_door")
    _add_right_door_leaf(right_door, material="safety_yellow", mesh_material="yellow_wire_mesh")

    latch_handle = model.part("latch_handle")
    _cylinder(latch_handle, "round_hub", 0.040, 0.020, (0.0, 0.0, 0.0), "black_hardware", rpy=(0.0, math.pi / 2.0, 0.0))
    _box(latch_handle, "grip_bar", (0.024, 0.034, 0.24), (-0.012, 0.0, -0.10), "black_hardware")
    _cylinder(latch_handle, "silver_pivot_cap", 0.020, 0.006, (-0.016, 0.0, 0.0), "galvanized_pin", rpy=(0.0, math.pi / 2.0, 0.0))

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=left_door,
        origin=Origin(xyz=(-0.780, -yh - 0.056, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=right_door,
        # At q=0 this leaf is shown swung open outward, matching the reference image.
        origin=Origin(xyz=(0.780, -yh - 0.056, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-1.57, upper=0.25),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=latch_handle,
        origin=Origin(xyz=(-0.050, -0.680, 1.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.57, upper=1.57),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cage = object_model.get_part("safety_cage")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    latch_handle = object_model.get_part("latch_handle")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_overlap(cage, left_door, axes="z", min_overlap=1.6, name="closed left door spans cage height")
    ctx.expect_overlap(cage, right_door, axes="z", min_overlap=1.6, name="open right door spans cage height")
    ctx.expect_contact(latch_handle, right_door, elem_a="round_hub", elem_b="latch_plate", contact_tol=0.002, name="handle hub is seated on latch plate")

    left_closed = ctx.part_world_aabb(left_door)
    with ctx.pose({left_hinge: 1.20}):
        left_open = ctx.part_world_aabb(left_door)
    ctx.check(
        "left leaf swings outward from front",
        left_closed is not None
        and left_open is not None
        and left_open[0][1] < left_closed[0][1] - 0.25,
        details=f"closed={left_closed}, open={left_open}",
    )

    right_open = ctx.part_world_aabb(right_door)
    with ctx.pose({right_hinge: -1.45}):
        right_closed = ctx.part_world_aabb(right_door)
    ctx.check(
        "right leaf can close from open reference pose",
        right_open is not None
        and right_closed is not None
        and right_open[0][1] < -1.20
        and right_closed[0][1] > -0.82
        and right_closed[0][0] < 0.12,
        details=f"open={right_open}, closed={right_closed}",
    )

    handle_rest = ctx.part_world_aabb(latch_handle)
    with ctx.pose({handle_pivot: math.pi / 2.0}):
        handle_turn = ctx.part_world_aabb(latch_handle)
    ctx.check(
        "latch handle rotates about door-mounted pivot",
        handle_rest is not None
        and handle_turn is not None
        and (handle_rest[1][2] - handle_rest[0][2]) > 0.18
        and (handle_turn[1][1] - handle_turn[0][1]) > 0.18,
        details=f"rest={handle_rest}, turned={handle_turn}",
    )

    return ctx.report()


object_model = build_object_model()
