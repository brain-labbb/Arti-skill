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


YELLOW = Material("powder_coated_safety_yellow", rgba=(1.0, 0.68, 0.02, 1.0))
WIRE = Material("galvanized_wire_mesh", rgba=(0.58, 0.58, 0.54, 1.0))
PLATE = Material("dark_bolt_heads", rgba=(0.08, 0.08, 0.075, 1.0))
FLOOR = Material("brushed_steel_base_plate", rgba=(0.60, 0.62, 0.62, 1.0))
MESH_FRAME_BITE = 0.105


def _box(part, name: str, size, xyz, material=YELLOW, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material=YELLOW, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_xz_mesh_panel(part, prefix: str, x0: float, x1: float, y: float, z0: float, z1: float) -> None:
    """Wire mesh in an XZ plane. The rods reach into the yellow perimeter frame."""
    wire = 0.009
    width = x1 - x0
    height = z1 - z0
    cx = (x0 + x1) * 0.5
    cz = (z0 + z1) * 0.5
    for i in range(9):
        x = x0 + width * i / 8.0
        _box(part, f"{prefix}_wire_v_{i}", (wire, wire, height + 2.0 * MESH_FRAME_BITE), (x, y, cz), WIRE)
    for j in range(10):
        z = z0 + height * j / 9.0
        _box(part, f"{prefix}_wire_h_{j}", (width + 2.0 * MESH_FRAME_BITE, wire, wire), (cx, y, z), WIRE)


def _add_yz_mesh_panel(part, prefix: str, x: float, y0: float, y1: float, z0: float, z1: float) -> None:
    """Wire mesh in a YZ plane."""
    wire = 0.009
    depth = y1 - y0
    height = z1 - z0
    cy = (y0 + y1) * 0.5
    cz = (z0 + z1) * 0.5
    for i in range(8):
        y = y0 + depth * i / 7.0
        _box(part, f"{prefix}_wire_v_{i}", (wire, wire, height + 2.0 * MESH_FRAME_BITE), (x, y, cz), WIRE)
    for j in range(10):
        z = z0 + height * j / 9.0
        _box(part, f"{prefix}_wire_h_{j}", (wire, depth + 2.0 * MESH_FRAME_BITE, wire), (x, cy, z), WIRE)


def _add_door_panel(part, prefix: str, side: float, width: float, height: float) -> None:
    """A framed yellow mesh door whose local origin is its vertical hinge axis."""
    tube = 0.045
    wire = 0.008
    y_thick = 0.042
    x_mid = side * width * 0.5
    rail_width = width - 0.040
    rail_mid = side * (0.040 + rail_width * 0.5)
    # Outer frame and two waist crossbars.
    # The hinge stile sits just inboard of the pin axis, leaving clearance for
    # alternating fixed hinge knuckles carried by the cage frame.
    _box(part, f"{prefix}_hinge_stile", (tube, y_thick, height), (side * (tube * 0.5 + 0.018), 0.0, height * 0.5), YELLOW)
    _box(part, f"{prefix}_meeting_stile", (tube, y_thick, height), (side * (width - tube * 0.5), 0.0, height * 0.5), YELLOW)
    _box(part, f"{prefix}_bottom_rail", (rail_width, y_thick, tube), (rail_mid, 0.0, tube * 0.5), YELLOW)
    _box(part, f"{prefix}_top_rail", (rail_width, y_thick, tube), (rail_mid, 0.0, height - tube * 0.5), YELLOW)
    _box(part, f"{prefix}_lower_rail", (rail_width, y_thick, tube * 0.78), (rail_mid, 0.0, height * 0.39), YELLOW)
    _box(part, f"{prefix}_upper_rail", (rail_width, y_thick, tube * 0.78), (rail_mid, 0.0, height * 0.68), YELLOW)

    # Welded wire infill, slightly behind the front face but intersecting the frame.
    mesh_x0 = tube
    mesh_x1 = width - tube
    mesh_h = height - 2.0 * tube
    for i in range(6):
        x = side * (mesh_x0 + (mesh_x1 - mesh_x0) * (i + 1) / 7.0)
        _box(part, f"{prefix}_mesh_v_{i}", (wire, wire, mesh_h + 0.090), (x, -0.003, height * 0.5), WIRE)
    for j in range(9):
        z = tube + mesh_h * (j + 1) / 10.0
        _box(part, f"{prefix}_mesh_h_{j}", (mesh_x1 - mesh_x0 + 0.090, wire, wire), (x_mid, -0.003, z), WIRE)

    # Exposed hinge knuckles on the moving leaf.
    for k, z in enumerate((0.22, 0.72, 1.22)):
        _cyl(part, f"{prefix}_hinge_barrel_{k}", 0.018, 0.22, (0.0, -0.004, z), YELLOW)
        _box(part, f"{prefix}_hinge_leaf_{k}", (0.055, 0.014, 0.16), (side * 0.036, -0.027, z), YELLOW)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="industrial_safety_cage",
        meta={
            "classification_note": "The reference image and category both read as an industrial safety cage / machine guard.",
        },
    )

    frame = model.part("fixed_frame")

    width = 3.20
    depth = 1.50
    height = 2.20
    tube = 0.055
    front_y = -depth * 0.5
    rear_y = depth * 0.5
    left_x = -width * 0.5
    right_x = width * 0.5

    # Grounded base plates with visible bolt heads.
    post_xy = [
        (left_x, front_y),
        (-0.82, front_y),
        (0.82, front_y),
        (right_x, front_y),
        (left_x, rear_y),
        (-0.82, rear_y),
        (0.82, rear_y),
        (right_x, rear_y),
    ]
    for i, (x, y) in enumerate(post_xy):
        _box(frame, f"base_plate_{i}", (0.20, 0.16, 0.020), (x, y, 0.010), FLOOR)
        _box(frame, f"post_{i}", (tube, tube, height), (x, y, height * 0.5 + 0.020), YELLOW)
        for sx in (-0.052, 0.052):
            for sy in (-0.040, 0.040):
                _cyl(frame, f"bolt_{i}_{sx}_{sy}", 0.010, 0.012, (x + sx, y + sy, 0.025), PLATE)

    # Front and rear perimeter rails plus mid-height protective rails.
    for y_name, y in (("front", front_y), ("rear", rear_y)):
        for z_name, z in (("base", 0.18), ("top", 2.13)):
            _box(frame, f"{y_name}_{z_name}_rail", (width + tube, tube, tube), (0.0, y, z), YELLOW)
        if y_name == "rear":
            _box(frame, "rear_waist_rail", (width + tube, tube, tube), (0.0, y, 1.05), YELLOW)
            _box(frame, "rear_upper_guard_rail", (width + tube, tube * 0.80, tube * 0.80), (0.0, y, 1.78), YELLOW)
        else:
            for side_name, x in (("left", -1.24), ("right", 1.24)):
                _box(frame, f"front_{side_name}_waist_rail", (0.72, tube, tube), (x, y, 1.05), YELLOW)
                _box(frame, f"front_{side_name}_upper_guard_rail", (0.72, tube * 0.80, tube * 0.80), (x, y, 1.78), YELLOW)

    # Side rails and open-top rectangular guard frame.
    for x_name, x in (("side_0", left_x), ("side_1", right_x)):
        for z_name, z in (("base", 0.18), ("waist", 1.05), ("top", 2.13)):
            _box(frame, f"{x_name}_{z_name}_rail", (tube, depth + tube, tube), (x, 0.0, z), YELLOW)
        _box(frame, f"{x_name}_upper_guard_rail", (tube * 0.80, depth + tube, tube * 0.80), (x, 0.0, 1.78), YELLOW)

    # Door opening jambs and a raised threshold/overhead lintel around the two access doors.
    for x in (-0.82, 0.82):
        _box(frame, f"front_door_jamb_{x}", (tube, tube, 1.72), (x, front_y, 1.06), YELLOW)
    _box(frame, "front_door_lintel", (1.72, tube, tube), (0.0, front_y, 1.88), YELLOW)
    _box(frame, "front_door_threshold", (1.72, tube, tube), (0.0, front_y, 0.18), YELLOW)

    # Fixed half of the exposed barrel hinges. These knuckles share the hinge axis
    # with the doors and touch the moving knuckles end-to-end, giving the doors a
    # visible mechanical support path without fusing them into the frame.
    for side_name, hx, leaf_cx in (("door_0", -0.76, -0.79), ("door_1", 0.76, 0.79)):
        for k, (z, length) in enumerate(((0.3275, 0.065), (0.72, 0.28), (1.22, 0.28), (1.67, 0.18))):
            _cyl(frame, f"{side_name}_fixed_hinge_knuckle_{k}", 0.018, length, (hx, front_y - 0.044, z), YELLOW)
            _box(frame, f"{side_name}_fixed_hinge_leaf_{k}", (0.09, 0.014, length * 0.72), (leaf_cx, front_y - 0.029, z), YELLOW)

    # Fixed mesh panels around the sides and rear; the center front opening is left for the hinged doors.
    mesh_bottom = 0.28
    mesh_top = 2.03
    _add_xz_mesh_panel(frame, "front_panel_0", left_x + 0.06, -0.88, front_y - 0.002, mesh_bottom, mesh_top)
    _add_xz_mesh_panel(frame, "front_panel_1", 0.88, right_x - 0.06, front_y - 0.002, mesh_bottom, mesh_top)
    for i, (x0, x1) in enumerate(((left_x + 0.06, -0.84), (-0.78, 0.0), (0.0, 0.78), (0.84, right_x - 0.06))):
        _add_xz_mesh_panel(frame, f"rear_panel_{i}", x0, x1, rear_y + 0.002, mesh_bottom, mesh_top)
    for i, x in enumerate((left_x - 0.002, right_x + 0.002)):
        _add_yz_mesh_panel(frame, f"side_panel_{i}", x, front_y + 0.06, rear_y - 0.06, mesh_bottom, mesh_top)

    # Small top lifting/handling eyes made from square rod loops, connected to the top rail.
    for i, x in enumerate((-1.35, 0.0, 1.35)):
        _box(frame, f"lift_eye_{i}_foot_0", (0.018, 0.018, 0.095), (x - 0.035, front_y, 2.205), YELLOW)
        _box(frame, f"lift_eye_{i}_foot_1", (0.018, 0.018, 0.095), (x + 0.035, front_y, 2.205), YELLOW)
        _box(frame, f"lift_eye_{i}_top", (0.088, 0.018, 0.018), (x, front_y, 2.252), YELLOW)

    # Two outward-opening access doors in the front opening.
    door_h = 1.55
    door_w = 0.74
    door_0 = model.part("door_0")
    _add_door_panel(door_0, "door_0", 1.0, door_w, door_h)
    _box(door_0, "latch_mount_plate", (0.13, 0.017, 0.16), (0.705, -0.028, 0.82), PLATE)
    door_1 = model.part("door_1")
    _add_door_panel(door_1, "door_1", -1.0, door_w, door_h)
    # A receiving keeper plate on the opposite door for the rotating latch.
    _box(door_1, "latch_keeper", (0.11, 0.018, 0.13), (-0.69, -0.029, 0.82), PLATE)

    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(-0.76, front_y - 0.040, 0.25)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "frame_to_door_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(0.76, front_y - 0.040, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Rotating latch/handle mounted to the leading edge of the first door.
    latch = model.part("latch_handle")
    _cyl(latch, "round_pivot", 0.040, 0.012, (0.0, -0.006, 0.0), PLATE, rpy=(math.pi / 2.0, 0.0, 0.0))
    _box(latch, "vertical_grip", (0.035, 0.018, 0.26), (0.0, -0.020, 0.0), PLATE)
    _box(latch, "latch_tongue", (0.20, 0.016, 0.026), (0.095, -0.022, 0.0), PLATE)
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door_0,
        child=latch,
        origin=Origin(xyz=(0.705, -0.0365, 0.82)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.5708),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fixed_frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    latch = object_model.get_part("latch_handle")
    joint_0 = object_model.get_articulation("frame_to_door_0")
    joint_1 = object_model.get_articulation("frame_to_door_1")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.check(
        "safety cage has articulated doors and latch",
        frame is not None and door_0 is not None and door_1 is not None and latch is not None,
        details="Expected a fixed yellow frame, two hinged access doors, and a latch handle.",
    )
    ctx.expect_overlap(door_0, door_1, axes="z", min_overlap=1.20, name="paired doors share a tall access opening")
    ctx.expect_gap(
        door_0,
        frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.08,
        positive_elem="door_0_bottom_rail",
        negative_elem="front_door_threshold",
        name="door bottoms sit just above threshold",
    )

    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({joint_0: 1.10, joint_1: 1.10}):
        open_0 = ctx.part_world_aabb(door_0)
        open_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "both access doors swing outward",
        closed_0 is not None
        and closed_1 is not None
        and open_0 is not None
        and open_1 is not None
        and open_0[0][1] < closed_0[0][1] - 0.20
        and open_1[0][1] < closed_1[0][1] - 0.20,
        details=f"closed_0={closed_0}, open_0={open_0}, closed_1={closed_1}, open_1={open_1}",
    )

    latch_closed = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 1.5708}):
        latch_rotated = ctx.part_world_aabb(latch)
    ctx.check(
        "latch handle rotates from vertical to release position",
        latch_closed is not None
        and latch_rotated is not None
        and (latch_closed[1][2] - latch_closed[0][2]) > (latch_closed[1][0] - latch_closed[0][0])
        and (latch_rotated[1][0] - latch_rotated[0][0]) > (latch_rotated[1][2] - latch_rotated[0][2]),
        details=f"closed={latch_closed}, rotated={latch_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
