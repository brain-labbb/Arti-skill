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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_pose(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that aligns a local +Z cylinder from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")

    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_tube(part, name: str, p0, p1, radius: float, material: Material) -> None:
    origin, length = _tube_pose(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_ball(part, name: str, xyz, radius: float, material: Material) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def _add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="folding_camp_stool",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Backless X-frame folding camp stool with olive-green seat and scissor-brace folding mechanism.",
            "variant": "backless_stool_olive",
        },
    )

    # Materials — olive-green seat colorway (companion variation ⑥)
    black_fabric = model.material("black_oxford_fabric", rgba=(0.012, 0.012, 0.014, 1.0))
    olive_fabric = model.material("olive_green_fabric", rgba=(0.30, 0.38, 0.18, 1.0))
    metal = model.material("speckled_dark_tubular_steel", rgba=(0.43, 0.43, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet_heads", rgba=(0.86, 0.88, 0.86, 1.0))

    # ────────────────────────────────────────────────────────────────
    # Square stool footprint and seat height
    # ────────────────────────────────────────────────────────────────
    left_x = -0.25
    right_x = 0.25
    front_y = -0.25
    rear_y = 0.25
    seat_z = 0.44
    tube_r = 0.014

    frame = model.part("chair_frame")

    # ── Feet and rivets ──
    for x, y, suffix in (
        (left_x, front_y, "front_0"),
        (right_x, front_y, "front_1"),
        (left_x, rear_y, "rear_0"),
        (right_x, rear_y, "rear_1"),
    ):
        _add_box(frame, f"{suffix}_foot", (0.090, 0.065, 0.024), (x, y, 0.012), black_plastic)
        _add_ball(frame, f"{suffix}_foot_rivet", (x, y, 0.028), 0.009, silver)

    # ── Four legs (all to seat height — no tall back posts) ──
    _add_tube(frame, "front_leg_0", (left_x, front_y, 0.025), (left_x, front_y, seat_z), tube_r, metal)
    _add_tube(frame, "front_leg_1", (right_x, front_y, 0.025), (right_x, front_y, seat_z), tube_r, metal)
    _add_tube(frame, "rear_leg_0", (left_x, rear_y, 0.025), (left_x, rear_y, seat_z), tube_r, metal)
    _add_tube(frame, "rear_leg_1", (right_x, rear_y, 0.025), (right_x, rear_y, seat_z), tube_r, metal)

    # ── Seat rails (uniform height, square perimeter) ──
    _add_tube(frame, "front_seat_rail", (left_x, front_y, seat_z), (right_x, front_y, seat_z), tube_r, metal)
    _add_tube(frame, "rear_seat_rail", (left_x, rear_y, seat_z), (right_x, rear_y, seat_z), tube_r, metal)
    _add_tube(frame, "side_seat_rail_0", (left_x, front_y, seat_z), (left_x, rear_y, seat_z), tube_r, metal)
    _add_tube(frame, "side_seat_rail_1", (right_x, front_y, seat_z), (right_x, rear_y, seat_z), tube_r, metal)

    # ── Fixed X-brace members (stationary diagonals on the frame) ──
    _add_tube(frame, "front_fixed_cross", (left_x, front_y + 0.016, seat_z), (right_x, front_y + 0.016, 0.065), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_0", (left_x + 0.045, front_y, seat_z), (left_x + 0.045, rear_y, 0.065), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_1", (right_x - 0.045, front_y, seat_z), (right_x - 0.045, rear_y, 0.065), tube_r, metal)

    # ── Lower stabilizer crosses ──
    _add_tube(frame, "front_lower_cross", (left_x, front_y, 0.09), (right_x, front_y, 0.09), tube_r, metal)
    _add_tube(frame, "rear_lower_cross", (left_x, rear_y, 0.09), (right_x, rear_y, 0.09), tube_r, metal)

    # ── Pivot pins and support brackets for the three scissor braces ──
    front_pivot_y = front_y - 0.060
    front_pivot_z = 0.260
    side_pivot_z = 0.280

    _add_tube(frame, "front_pivot_pin", (-0.002, front_pivot_y - 0.030, front_pivot_z), (-0.002, front_pivot_y + 0.030, front_pivot_z), 0.016, black_plastic)
    _add_tube(frame, "front_pivot_support", (0.0, front_pivot_y + 0.024, front_pivot_z), (0.0, front_y, seat_z), 0.009, black_plastic)
    _add_tube(frame, "side_pivot_pin_0", (left_x - 0.034, -0.020, side_pivot_z), (left_x + 0.024, -0.020, side_pivot_z), 0.016, black_plastic)
    _add_tube(frame, "side_pivot_pin_1", (right_x - 0.024, -0.020, side_pivot_z), (right_x + 0.034, -0.020, side_pivot_z), 0.016, black_plastic)
    # Bracket tubes connect pivot hardware down to the front legs
    _add_tube(frame, "side_pivot_support_0", (left_x + 0.024, -0.020, side_pivot_z), (left_x, front_y, 0.18), 0.008, black_plastic)
    _add_tube(frame, "side_pivot_support_1", (right_x - 0.024, -0.020, side_pivot_z), (right_x, front_y, 0.18), 0.008, black_plastic)
    for xyz, name in (
        ((-0.002, front_pivot_y - 0.030, front_pivot_z), "front_pivot_rivet"),
        ((left_x - 0.034, -0.020, side_pivot_z), "side_pivot_rivet_0"),
        ((right_x + 0.034, -0.020, side_pivot_z), "side_pivot_rivet_1"),
    ):
        _add_ball(frame, name, xyz, 0.009, silver)

    # ── Sling seat: olive-green center, black edge panels, front roll ──
    _add_box(frame, "seat_black_left", (0.10, 0.52, 0.018), (-0.175, 0.0, seat_z + 0.010), black_fabric)
    _add_box(frame, "seat_black_right", (0.10, 0.52, 0.018), (0.175, 0.0, seat_z + 0.010), black_fabric)
    _add_box(frame, "seat_gray_center", (0.27, 0.52, 0.016), (0.0, 0.0, seat_z + 0.012), olive_fabric)
    _add_box(frame, "seat_front_roll", (0.52, 0.038, 0.032), (0.0, front_y - 0.012, seat_z + 0.020), black_fabric)
    _add_box(frame, "seat_rear_lap", (0.48, 0.032, 0.028), (0.0, rear_y + 0.008, seat_z + 0.012), black_fabric)

    # ────────────────────────────────────────────────────────────────
    # Articulated scissor braces (child parts)
    # ────────────────────────────────────────────────────────────────

    # Front cross brace — pivots around the front X intersection
    front_brace = model.part("front_cross_brace")
    _add_tube(front_brace, "front_moving_tube", (-0.22, -0.060, -0.185), (0.22, -0.060, 0.175), tube_r, metal)
    _add_tube(front_brace, "front_pivot_collar", (0.0, -0.048, 0.0), (0.0, 0.016, 0.0), 0.018, black_plastic)
    _add_ball(front_brace, "front_brace_end_0", (-0.22, -0.060, -0.185), 0.016, black_plastic)
    _add_ball(front_brace, "front_brace_end_1", (0.22, -0.060, 0.175), 0.016, black_plastic)

    # Left side cross brace
    side_brace_0 = model.part("side_cross_brace_0")
    _add_tube(side_brace_0, "side_moving_tube_0", (-0.060, -0.22, -0.210), (-0.060, 0.22, 0.155), tube_r, metal)
    _add_tube(side_brace_0, "side_pivot_collar_0", (-0.048, 0.0, 0.0), (0.016, 0.0, 0.0), 0.018, black_plastic)
    _add_ball(side_brace_0, "side_brace_end_0a", (-0.060, -0.22, -0.210), 0.016, black_plastic)
    _add_ball(side_brace_0, "side_brace_end_0b", (-0.060, 0.22, 0.155), 0.016, black_plastic)

    # Right side cross brace
    side_brace_1 = model.part("side_cross_brace_1")
    _add_tube(side_brace_1, "side_moving_tube_1", (0.060, -0.22, -0.210), (0.060, 0.22, 0.155), tube_r, metal)
    _add_tube(side_brace_1, "side_pivot_collar_1", (-0.016, 0.0, 0.0), (0.048, 0.0, 0.0), 0.018, black_plastic)
    _add_ball(side_brace_1, "side_brace_end_1a", (0.060, -0.22, -0.210), 0.016, black_plastic)
    _add_ball(side_brace_1, "side_brace_end_1b", (0.060, 0.22, 0.155), 0.016, black_plastic)

    # ────────────────────────────────────────────────────────────────
    # Articulations — three revolute scissor pivots
    # ────────────────────────────────────────────────────────────────
    model.articulation(
        "frame_to_front_cross",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_brace,
        origin=Origin(xyz=(0.0, front_pivot_y, front_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )
    model.articulation(
        "frame_to_side_cross_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_brace_0,
        origin=Origin(xyz=(left_x - 0.006, -0.020, side_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )
    model.articulation(
        "frame_to_side_cross_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_brace_1,
        origin=Origin(xyz=(right_x + 0.006, -0.020, side_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    front = object_model.get_part("front_cross_brace")
    side0 = object_model.get_part("side_cross_brace_0")
    side1 = object_model.get_part("side_cross_brace_1")
    front_joint = object_model.get_articulation("frame_to_front_cross")
    side_joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    side_joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # ── Structural change assertion: backless stool has no backrest or arm parts ──
    frame_visual_names = {v.name for v in frame.visuals}
    ctx.check(
        "no backrest visuals on stool frame",
        not any(n.startswith("back_") for n in frame_visual_names),
        details=f"Found back visuals: {[n for n in frame_visual_names if n.startswith('back_')]}",
    )
    ctx.check(
        "no armrest visuals on stool frame",
        not any(n.startswith("arm_") for n in frame_visual_names),
        details=f"Found arm visuals: {[n for n in frame_visual_names if n.startswith('arm_')]}",
    )
    ctx.check(
        "no tall rear back posts — stool has short rear_leg_0 and rear_leg_1",
        "rear_leg_0" in frame_visual_names and "rear_leg_1" in frame_visual_names
        and "rear_back_post_0" not in frame_visual_names
        and "rear_back_post_1" not in frame_visual_names,
        details=f"rear visuals: {[n for n in frame_visual_names if 'rear' in n]}",
    )
    ctx.check(
        "olive-green seat center present",
        "seat_gray_center" in frame_visual_names,
        details=f"seat visuals: {[n for n in frame_visual_names if 'seat' in n]}",
    )

    # ── Provenance metadata ──
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # ── Pivot capture allowances ──
    ctx.allow_overlap(
        frame, front,
        elem_a="front_pivot_pin", elem_b="front_pivot_collar",
        reason="The moving scissor brace collar is intentionally captured around the fixed front pivot pin.",
    )
    ctx.allow_overlap(
        frame, side0,
        elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, side1,
        elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, side0,
        elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, side1,
        elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, front,
        elem_a="front_pivot_rivet", elem_b="front_pivot_collar",
        reason="The front rivet head intentionally captures the rotating scissor collar on the pivot.",
    )
    ctx.allow_overlap(
        frame, side0,
        elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, side1,
        elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )

    # ── Pivot retention checks ──
    ctx.expect_overlap(frame, front, axes="yz", min_overlap=0.015, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left side brace collar retained on pivot")
    ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right side brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.008, elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0", name="left rivet head clamps pivot collar")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.008, elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1", name="right rivet head clamps pivot collar")
    ctx.expect_overlap(frame, front, axes="xz", min_overlap=0.008, elem_a="front_pivot_rivet", elem_b="front_pivot_collar", name="front rivet head captures pivot collar")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.008, elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0", name="left rivet passes through moving brace")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.008, elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1", name="right rivet passes through moving brace")
    ctx.expect_gap(frame, front, axis="z", max_penetration=0.05, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front pivot overlap remains localized")

    # ── Mirrored side folding pivots ──
    ctx.check(
        "side folding pivots are mirrored",
        tuple(side_joint_0.axis) == (1.0, 0.0, 0.0)
        and tuple(side_joint_1.axis) == (-1.0, 0.0, 0.0)
        and side_joint_0.motion_limits.lower == 0.0
        and side_joint_1.motion_limits.lower == 0.0
        and side_joint_0.motion_limits.upper > 0.5
        and side_joint_1.motion_limits.upper > 0.5,
        details=f"left_axis={side_joint_0.axis} right_axis={side_joint_1.axis}",
    )

    # ── Folding motion proof ──
    closed_aabb = ctx.part_world_aabb(front)
    side0_closed_aabb = ctx.part_world_aabb(side0)
    side1_closed_aabb = ctx.part_world_aabb(side1)
    with ctx.pose({front_joint: 0.45, side_joint_0: 0.45, side_joint_1: 0.45}):
        folded_aabb = ctx.part_world_aabb(front)
        side0_folded_aabb = ctx.part_world_aabb(side0)
        side1_folded_aabb = ctx.part_world_aabb(side1)
        ctx.expect_overlap(frame, front, axes="yz", min_overlap=0.012, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front pivot still captured while folding")
        ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.012, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left pivot still captured while folding")
        ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.012, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right pivot still captured while folding")

    ctx.check(
        "front cross brace changes angle when folding",
        closed_aabb is not None
        and folded_aabb is not None
        and abs((folded_aabb[1][2] - folded_aabb[0][2]) - (closed_aabb[1][2] - closed_aabb[0][2])) > 0.020,
        details=f"rest={closed_aabb}, folded={folded_aabb}",
    )
    ctx.check(
        "side braces fold as a mirrored pair",
        side0_closed_aabb is not None
        and side1_closed_aabb is not None
        and side0_folded_aabb is not None
        and side1_folded_aabb is not None
        and abs((side0_folded_aabb[1][1] - side0_folded_aabb[0][1]) - (side0_closed_aabb[1][1] - side0_closed_aabb[0][1])) > 0.015
        and abs((side1_folded_aabb[1][1] - side1_folded_aabb[0][1]) - (side1_closed_aabb[1][1] - side1_closed_aabb[0][1])) > 0.015,
        details=f"left_rest={side0_closed_aabb}, left_folded={side0_folded_aabb}, right_rest={side1_closed_aabb}, right_folded={side1_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
