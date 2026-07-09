from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


FRAME_GRAY = _mat("painted_gray", (0.46, 0.49, 0.48, 1.0))
GALVANIZED = _mat("galvanized_silver", (0.78, 0.80, 0.78, 1.0))
SAFETY_YELLOW = _mat("safety_yellow", (1.0, 0.70, 0.05, 1.0))
PAYLOAD_RED = _mat("payload_red", (0.62, 0.05, 0.035, 1.0))
DARK_RUBBER = _mat("dark_rubber", (0.025, 0.025, 0.022, 1.0))


def _box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _rod_z(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(Cylinder(radius, length), origin=Origin(xyz=xyz), material=material, name=name)


def _rod_x(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius, length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_y(part, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius, length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foreground_wire_safety_cage")

    # Reference-like compact industrial cage proportions, in meters.
    width = 0.62
    depth = 0.50
    height = 0.72
    post = 0.045
    rail_h = 0.050
    top_z = height
    bottom_z = 0.035
    panel_mid_z = 0.38
    panel_h = 0.66
    mesh_r = 0.0038

    cage = model.part("cage_frame")

    # Gray structural base and top frame.
    _box(cage, "front_base_rail", (width + post, post, rail_h), (0.0, -depth / 2.0, bottom_z), FRAME_GRAY)
    _box(cage, "rear_base_rail", (width + post, post, rail_h), (0.0, depth / 2.0, bottom_z), FRAME_GRAY)
    _box(cage, "side_base_rail_0", (post, depth + post, rail_h), (-width / 2.0, 0.0, bottom_z), FRAME_GRAY)
    _box(cage, "side_base_rail_1", (post, depth + post, rail_h), (width / 2.0, 0.0, bottom_z), FRAME_GRAY)
    _box(cage, "front_top_rail", (width + post, post, post), (0.0, -depth / 2.0, top_z), FRAME_GRAY)
    _box(cage, "rear_top_rail", (width + post, post, post), (0.0, depth / 2.0, top_z), FRAME_GRAY)
    _box(cage, "side_top_rail_0", (post, depth + post, post), (-width / 2.0, 0.0, top_z), FRAME_GRAY)
    _box(cage, "side_top_rail_1", (post, depth + post, post), (width / 2.0, 0.0, top_z), FRAME_GRAY)

    # Four gray upright corner posts tie the bottom and top frames together.
    for ix, x in enumerate((-width / 2.0, width / 2.0)):
        for iy, y in enumerate((-depth / 2.0, depth / 2.0)):
            post_name = "front_left_post" if (ix == 0 and iy == 0) else f"corner_post_{ix}_{iy}"
            _box(cage, post_name, (post, post, height - 0.02), (x, y, height / 2.0 + 0.02), FRAME_GRAY)

    # Small leveling feet under the base frame, visibly carrying the cage.
    for ix, x in enumerate((-width / 2.0 + 0.030, width / 2.0 - 0.030)):
        for iy, y in enumerate((-depth / 2.0 + 0.030, depth / 2.0 - 0.030)):
            _rod_z(cage, f"foot_{ix}_{iy}", 0.030, 0.018, (x, y, 0.004), DARK_RUBBER)

    # Galvanized wire mesh sides and rear. Rods slightly enter the gray rails/posts
    # so every visible wire is mechanically captured instead of floating.
    side_y_positions = [-0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18]
    side_z_positions = [0.15, 0.22, 0.29, 0.36, 0.43, 0.50, 0.57, 0.64]
    for side_index, x in enumerate((-width / 2.0, width / 2.0)):
        side_name = "left_side" if side_index == 0 else "right_side"
        for i, y in enumerate(side_y_positions):
            _rod_z(cage, f"{side_name}_vertical_{i}", mesh_r, panel_h, (x, y, panel_mid_z), GALVANIZED)
        for i, z in enumerate(side_z_positions):
            _rod_y(cage, f"{side_name}_horizontal_{i}", mesh_r, depth, (x, 0.0, z), GALVANIZED)

    rear_x_positions = [-0.24, -0.18, -0.12, -0.06, 0.0, 0.06, 0.12, 0.18, 0.24]
    for i, x in enumerate(rear_x_positions):
        _rod_z(cage, f"rear_vertical_{i}", mesh_r, panel_h, (x, depth / 2.0, panel_mid_z), GALVANIZED)
    for i, z in enumerate(side_z_positions):
        _rod_x(cage, f"rear_horizontal_{i}", mesh_r, width, (0.0, depth / 2.0, z), GALVANIZED)

    # Roof mesh, caught between the top rails.
    for i, x in enumerate([-0.24, -0.16, -0.08, 0.0, 0.08, 0.16, 0.24]):
        _rod_y(cage, f"roof_front_back_{i}", mesh_r, depth, (x, 0.0, top_z + 0.004), GALVANIZED)
    for i, y in enumerate([-0.18, -0.10, -0.02, 0.06, 0.14, 0.22]):
        _rod_x(cage, f"roof_side_side_{i}", mesh_r, width, (0.0, y, top_z + 0.004), GALVANIZED)

    # Simple fixed keeper/hasp target on the front-right post for the open door.
    _box(cage, "latch_keeper", (0.018, 0.012, 0.070), (width / 2.0, -depth / 2.0 - 0.027, 0.41), GALVANIZED)
    _box(cage, "bottom_shelf", (width + 0.020, depth, 0.018), (0.0, 0.0, 0.051), GALVANIZED)

    # Red enclosed payload/cabinet inside the cage, resting on the bottom rails.
    payload = model.part("red_payload")
    _box(payload, "red_cabinet", (0.36, 0.26, 0.24), (0.0, 0.0, 0.0), PAYLOAD_RED)
    _box(payload, "top_cap", (0.38, 0.28, 0.025), (0.0, 0.0, 0.132), PAYLOAD_RED)
    _box(payload, "front_handle", (0.090, 0.012, 0.030), (0.0, -0.136, 0.035), GALVANIZED)
    model.articulation(
        "cage_to_payload",
        ArticulationType.FIXED,
        parent=cage,
        child=payload,
        origin=Origin(xyz=(0.045, 0.055, 0.180)),
    )

    # Yellow hinged access door. At q=0 it is open outward in front of the cage;
    # rotating the hinge by +90 degrees brings it across the front opening.
    door = model.part("access_door")
    door_h = 0.68
    door_w = 0.61
    bar = 0.035
    hinge_barrel_r = 0.012
    hinge_embed_depth = 0.004
    door_frame_x = 0.0
    hinge_stile_y = -(hinge_barrel_r + bar / 2.0)

    def door_axis_y(y: float) -> float:
        return hinge_stile_y + y

    _box(
        door,
        "hinge_stile",
        (bar, bar, door_h),
        (door_frame_x, door_axis_y(0.0), 0.0),
        SAFETY_YELLOW,
    )
    _box(
        door,
        "free_stile",
        (bar, bar, door_h),
        (door_frame_x, door_axis_y(-door_w), 0.0),
        SAFETY_YELLOW,
    )
    _box(
        door,
        "top_rail",
        (bar, door_w + bar, bar),
        (door_frame_x, door_axis_y(-door_w / 2.0), door_h / 2.0),
        SAFETY_YELLOW,
    )
    _box(
        door,
        "bottom_rail",
        (bar, door_w + bar, bar),
        (door_frame_x, door_axis_y(-door_w / 2.0), -door_h / 2.0),
        SAFETY_YELLOW,
    )
    _box(
        door,
        "middle_rail",
        (bar, door_w + bar, 0.026),
        (door_frame_x, door_axis_y(-door_w / 2.0), 0.030),
        SAFETY_YELLOW,
    )

    # Door wire mesh: rods penetrate the yellow border and cross each other.
    for i, y in enumerate([-0.08, -0.15, -0.22, -0.29, -0.36, -0.43, -0.50]):
        _rod_z(
            door,
            f"door_vertical_{i}",
            mesh_r,
            door_h - 0.020,
            (door_frame_x, door_axis_y(y), 0.0),
            GALVANIZED,
        )
    for i, z in enumerate([-0.23, -0.16, -0.09, -0.02, 0.05, 0.12, 0.19, 0.26]):
        _rod_y(
            door,
            f"door_horizontal_{i}",
            mesh_r,
            door_w - 0.035,
            (door_frame_x, door_axis_y(-door_w / 2.0), z),
            GALVANIZED,
        )

    # Visible hinge barrels are centered on the revolute joint axis.
    _rod_z(door, "upper_hinge_barrel", hinge_barrel_r, 0.105, (0.0, 0.0, 0.205), GALVANIZED)
    _rod_z(door, "lower_hinge_barrel", hinge_barrel_r, 0.105, (0.0, 0.0, -0.205), GALVANIZED)
    _box(
        door,
        "upper_hinge_leaf",
        (0.030, 0.030, 0.080),
        (door_frame_x, hinge_stile_y / 2.0, 0.205),
        GALVANIZED,
    )
    _box(
        door,
        "lower_hinge_leaf",
        (0.030, 0.030, 0.080),
        (door_frame_x, hinge_stile_y / 2.0, -0.205),
        GALVANIZED,
    )
    _box(
        door,
        "latch_plate",
        (0.010, 0.070, 0.060),
        (door_frame_x - 0.022, door_axis_y(-door_w + 0.015), 0.030),
        GALVANIZED,
    )

    hinge_axis_x = -width / 2.0
    hinge_axis_y = -depth / 2.0 - post / 2.0 - hinge_barrel_r + hinge_embed_depth
    hinge_origin = Origin(
        xyz=(hinge_axis_x, hinge_axis_y, 0.380)
    )
    model.articulation(
        "cage_to_door",
        ArticulationType.REVOLUTE,
        parent=cage,
        child=door,
        origin=hinge_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
    )

    # Small rotating latch/handle, mounted on the free edge of the door.
    latch = model.part("latch_handle")
    _rod_x(latch, "pivot_pin", 0.010, 0.024, (-0.012, 0.0, 0.0), GALVANIZED)
    _box(latch, "drop_hasp", (0.010, 0.024, 0.082), (-0.024, 0.0, -0.044), GALVANIZED)
    latch.visual(
        Sphere(0.018),
        origin=Origin(xyz=(-0.024, 0.0, -0.092)),
        material=GALVANIZED,
        name="pull_knob",
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(door_frame_x - 0.024, door_axis_y(-door_w + 0.015), 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cage = object_model.get_part("cage_frame")
    door = object_model.get_part("access_door")
    latch = object_model.get_part("latch_handle")
    payload = object_model.get_part("red_payload")
    door_joint = object_model.get_articulation("cage_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.expect_contact(
        door,
        cage,
        elem_a="upper_hinge_barrel",
        elem_b="front_left_post",
        contact_tol=0.002,
        name="upper hinge barrel is embedded against cage post",
    )
    ctx.expect_contact(
        door,
        cage,
        elem_a="lower_hinge_barrel",
        elem_b="front_left_post",
        contact_tol=0.002,
        name="lower hinge barrel is embedded against cage post",
    )
    ctx.expect_within(
        payload,
        cage,
        axes="xy",
        inner_elem="red_cabinet",
        outer_elem="rear_base_rail",
        margin=0.35,
        name="red payload remains inside cage footprint",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="pivot_pin",
        elem_b="latch_plate",
        contact_tol=0.004,
        name="latch pivot is mounted on door latch plate",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    rest_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({door_joint: pi / 2.0}):
        closed_door_aabb = ctx.part_world_aabb(door)
        ctx.expect_overlap(
            door,
            cage,
            axes="xz",
            elem_a="free_stile",
            elem_b="front_top_rail",
            min_overlap=0.020,
            name="door swings across the front opening when closed",
        )
    with ctx.pose({latch_joint: 0.55}):
        swung_latch_aabb = ctx.part_world_aabb(latch)

    ctx.check(
        "rest pose presents an outward open door",
        rest_door_aabb is not None
        and rest_door_aabb[0][1] < -0.80
        and closed_door_aabb is not None
        and closed_door_aabb[0][1] > -0.33,
        details=f"rest_door_aabb={rest_door_aabb}, closed_door_aabb={closed_door_aabb}",
    )
    ctx.check(
        "latch handle has a real rotational pose",
        rest_latch_aabb is not None
        and swung_latch_aabb is not None
        and abs(swung_latch_aabb[1][1] - rest_latch_aabb[1][1]) > 0.020,
        details=f"rest_latch_aabb={rest_latch_aabb}, swung_latch_aabb={swung_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
