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


def _add_box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="directors_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Classic folding director's chair with rigid rectangular side frames, X-scissor braces, taut canvas seat and back, and hardwood arm rails.",
        },
    )

    # Companion variation ⑥: natural hardwood arm rails + tan/olive canvas
    wood = model.material("natural_hardwood", rgba=(0.55, 0.36, 0.18, 1.0))
    olive_canvas = model.material("olive_tan_canvas", rgba=(0.50, 0.50, 0.34, 1.0))
    metal = model.material("dark_steel_tube", rgba=(0.30, 0.30, 0.28, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.02, 1.0))
    silver = model.material("silver_rivet", rgba=(0.82, 0.84, 0.82, 1.0))

    # ------------------------------------------------------------------ root
    frame = model.part("chair_frame")

    # Key real-world dimensions for a director's chair
    left_x = -0.25
    right_x = 0.25
    front_y = -0.22
    rear_y = 0.22
    foot_z = 0.015
    seat_z = 0.44
    arm_z = 0.63
    back_top_z = 0.92
    tube_r = 0.014

    # X-brace geometry parameters
    pivot_z = (0.16 + 0.51) * 0.5  # 0.335 – crossing height
    bx = 0.21  # half-span of the X diagonals
    bz_lo = 0.16
    bz_hi = 0.51
    dy = 0.016  # Y offset separating fixed and moving diagonals

    # =========== RIGID RECTANGULAR SIDE FRAMES ===========

    # Left side frame (front leg + rear post + top rail + seat rail + stretcher)
    _add_tube(frame, "left_front_leg",
              (left_x, front_y, foot_z + 0.01), (left_x, front_y, arm_z), tube_r, metal)
    _add_tube(frame, "left_rear_post",
              (left_x, rear_y, foot_z + 0.01), (left_x, rear_y + 0.02, back_top_z), tube_r, metal)
    _add_tube(frame, "left_top_rail",
              (left_x, front_y, arm_z), (left_x, rear_y, arm_z), tube_r, metal)
    _add_tube(frame, "left_seat_rail",
              (left_x, front_y, seat_z), (left_x, rear_y, seat_z), tube_r, metal)
    _add_tube(frame, "left_bottom_stretcher",
              (left_x, front_y, 0.12), (left_x, rear_y, 0.12), tube_r, metal)

    # Right side frame (mirror)
    _add_tube(frame, "right_front_leg",
              (right_x, front_y, foot_z + 0.01), (right_x, front_y, arm_z), tube_r, metal)
    _add_tube(frame, "right_rear_post",
              (right_x, rear_y, foot_z + 0.01), (right_x, rear_y + 0.02, back_top_z), tube_r, metal)
    _add_tube(frame, "right_top_rail",
              (right_x, front_y, arm_z), (right_x, rear_y, arm_z), tube_r, metal)
    _add_tube(frame, "right_seat_rail",
              (right_x, front_y, seat_z), (right_x, rear_y, seat_z), tube_r, metal)
    _add_tube(frame, "right_bottom_stretcher",
              (right_x, front_y, 0.12), (right_x, rear_y, 0.12), tube_r, metal)

    # =========== HORIZONTAL CROSS MEMBERS ===========
    _add_tube(frame, "front_lower_cross",
              (left_x, front_y, 0.12), (right_x, front_y, 0.12), tube_r, metal)
    _add_tube(frame, "rear_lower_cross",
              (left_x, rear_y, 0.12), (right_x, rear_y, 0.12), tube_r, metal)
    _add_tube(frame, "top_back_rail",
              (left_x, rear_y + 0.02, back_top_z), (right_x, rear_y + 0.02, back_top_z), tube_r, metal)
    _add_tube(frame, "back_lower_rail",
              (left_x, rear_y + 0.01, 0.54), (right_x, rear_y + 0.01, 0.54), tube_r, metal)

    # =========== FIXED X-BRACE DIAGONALS (one per side, in frame) ===========
    # Front fixed diagonal: left-low → right-high
    _add_tube(frame, "front_fixed_diagonal",
              (-bx, front_y + dy, bz_lo), (bx, front_y + dy, bz_hi), tube_r, metal)
    # Rear fixed diagonal: left-low → right-high
    _add_tube(frame, "rear_fixed_diagonal",
              (-bx, rear_y + dy, bz_lo), (bx, rear_y + dy, bz_hi), tube_r, metal)

    # =========== PIVOT HARDWARE ===========
    # Front pivot pin (along Y, through both diagonals at the crossing)
    _add_tube(frame, "front_pivot_pin",
              (0.0, front_y - 0.035, pivot_z), (0.0, front_y + 0.035, pivot_z),
              0.010, black_plastic)
    # Rear pivot pin
    _add_tube(frame, "rear_pivot_pin",
              (0.0, rear_y - 0.035, pivot_z), (0.0, rear_y + 0.035, pivot_z),
              0.010, black_plastic)

    # Connecting brackets from pivot pins to the fixed diagonals for structural connectivity
    _add_tube(frame, "front_pivot_bracket",
              (0.0, front_y + 0.032, pivot_z), (0.0, front_y + dy + 0.002, pivot_z),
              0.010, metal)
    _add_tube(frame, "rear_pivot_bracket",
              (0.0, rear_y + 0.032, pivot_z), (0.0, rear_y + dy + 0.002, pivot_z),
              0.010, metal)

    # Connecting tubes from fixed diagonal endpoints to the lower cross members for connectivity
    _add_tube(frame, "front_diag_gusset_0",
              (-bx, front_y + dy, bz_lo), (-bx - 0.020, front_y, bz_lo - 0.030),
              0.008, metal)
    _add_tube(frame, "front_diag_gusset_1",
              (bx, front_y + dy, bz_hi), (bx + 0.020, front_y, bz_hi + 0.008),
              0.008, metal)
    _add_tube(frame, "rear_diag_gusset_0",
              (-bx, rear_y + dy, bz_lo), (-bx - 0.020, rear_y, bz_lo - 0.030),
              0.008, metal)
    _add_tube(frame, "rear_diag_gusset_1",
              (bx, rear_y + dy, bz_hi), (bx + 0.020, rear_y, bz_hi + 0.008),
              0.008, metal)

    # Rivet/washer heads at pin ends, clamping the collar from outside
    for y_base, prefix in ((front_y, "front"), (rear_y, "rear")):
        _add_box(frame, f"{prefix}_pivot_rivet_0",
                 (0.024, 0.024, 0.024), (0.0, y_base - 0.020, pivot_z), silver)
        _add_box(frame, f"{prefix}_pivot_rivet_1",
                 (0.024, 0.024, 0.024), (0.0, y_base + 0.020, pivot_z), silver)

    # =========== FEET (rubber caps) ===========
    for x, y, name in (
        (left_x, front_y, "left_front_foot"),
        (right_x, front_y, "right_front_foot"),
        (left_x, rear_y, "left_rear_foot"),
        (right_x, rear_y, "right_rear_foot"),
    ):
        _add_box(frame, name, (0.040, 0.040, 0.022), (x, y, foot_z), black_plastic)

    # =========== FLAT TAUT SEAT CANVAS ===========
    # Wide enough in X to contact side seat rails for connectivity;
    # shorter in Y to avoid the moving X-brace diagonal at the rear.
    _add_box(frame, "seat_gray_center",
             (0.50, 0.36, 0.012), (0.0, -0.02, seat_z + 0.006), olive_canvas)

    # =========== FLAT TAUT BACK CANVAS ===========
    back_center_z = (0.54 + back_top_z) * 0.5  # 0.73
    back_height = back_top_z - 0.54 - 0.04  # ~0.34
    # Widened to contact the rear posts for structural connectivity
    _add_box(frame, "back_gray_center",
             (0.50, 0.010, back_height), (0.0, rear_y + 0.012, back_center_z), olive_canvas)

    # =========== HARDWOOD ARMREST BOARDS ===========
    arm_len = abs(rear_y - front_y) - 0.04  # 0.40
    _add_box(frame, "arm_rest_top_0",
             (0.076, arm_len, 0.018), (left_x, 0.0, arm_z + 0.009), wood)
    _add_box(frame, "arm_rest_top_1",
             (0.076, arm_len, 0.018), (right_x, 0.0, arm_z + 0.009), wood)

    # =========== ARTICULATED X-BRACE MOVING MEMBERS ===========

    # Front X-brace (side_cross_brace_0): moving diagonal
    # Local frame origin = joint origin = (0, front_y, pivot_z) in world
    brace_0 = model.part("side_cross_brace_0")
    _add_tube(brace_0, "front_moving_diagonal",
              (bx, -dy, bz_lo - pivot_z), (-bx, -dy, bz_hi - pivot_z), tube_r, metal)
    _add_tube(brace_0, "front_pivot_collar",
              (0.0, -0.018, 0.0), (0.0, 0.018, 0.0), 0.018, black_plastic)
    _add_box(brace_0, "front_brace_end_0",
             (0.022, 0.022, 0.022), (bx, -dy, bz_lo - pivot_z), black_plastic)
    _add_box(brace_0, "front_brace_end_1",
             (0.022, 0.022, 0.022), (-bx, -dy, bz_hi - pivot_z), black_plastic)

    # Rear X-brace (side_cross_brace_1): moving diagonal
    # Local frame origin = joint origin = (0, rear_y, pivot_z) in world
    brace_1 = model.part("side_cross_brace_1")
    _add_tube(brace_1, "rear_moving_diagonal",
              (bx, -dy, bz_lo - pivot_z), (-bx, -dy, bz_hi - pivot_z), tube_r, metal)
    _add_tube(brace_1, "rear_pivot_collar",
              (0.0, -0.018, 0.0), (0.0, 0.018, 0.0), 0.018, black_plastic)
    _add_box(brace_1, "rear_brace_end_0",
             (0.022, 0.022, 0.022), (bx, -dy, bz_lo - pivot_z), black_plastic)
    _add_box(brace_1, "rear_brace_end_1",
             (0.022, 0.022, 0.022), (-bx, -dy, bz_hi - pivot_z), black_plastic)

    # =========== ARTICULATIONS ===========

    # Front X-brace: axis +Y so positive q rotates the moving diagonal toward
    # vertical (closing the X, narrowing the chair width → folding sideways).
    model.articulation(
        "frame_to_side_cross_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace_0,
        origin=Origin(xyz=(0.0, front_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )

    # Rear X-brace: same axis direction so both X-braces fold simultaneously.
    model.articulation(
        "frame_to_side_cross_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace_1,
        origin=Origin(xyz=(0.0, rear_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    brace_0 = object_model.get_part("side_cross_brace_0")
    brace_1 = object_model.get_part("side_cross_brace_1")
    joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # ---------- Overlap allowances for captured pivot hardware ----------

    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_pin", elem_b="front_pivot_collar",
        reason="The front X-brace pivot collar is intentionally captured around the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_pin", elem_b="rear_pivot_collar",
        reason="The rear X-brace pivot collar is intentionally captured around the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_pin", elem_b="front_moving_diagonal",
        reason="The front pivot pin passes through the moving diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_fixed_diagonal", elem_b="front_pivot_collar",
        reason="The front pivot collar wraps around both diagonals at the scissor crossing; the fixed diagonal is intentionally captured inside the collar.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_fixed_diagonal", elem_b="front_moving_diagonal",
        reason="The fixed and moving diagonals intentionally cross through each other at the scissor pivot point.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_fixed_diagonal", elem_b="front_brace_end_0",
        reason="The brace end cap is intentionally seated on the fixed diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_fixed_diagonal", elem_b="front_brace_end_1",
        reason="The brace end cap is intentionally seated on the fixed diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_pin", elem_b="rear_moving_diagonal",
        reason="The rear pivot pin passes through the moving diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_fixed_diagonal", elem_b="rear_pivot_collar",
        reason="The rear pivot collar wraps around both diagonals at the scissor crossing; the fixed diagonal is intentionally captured inside the collar.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_fixed_diagonal", elem_b="rear_moving_diagonal",
        reason="The fixed and moving diagonals intentionally cross through each other at the scissor pivot point.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_fixed_diagonal", elem_b="rear_brace_end_0",
        reason="The brace end cap is intentionally seated on the fixed diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_fixed_diagonal", elem_b="rear_brace_end_1",
        reason="The brace end cap is intentionally seated on the fixed diagonal tube at the scissor crossing.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_rivet_0", elem_b="front_pivot_collar",
        reason="Front pivot rivet head clamps the outside of the scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_rivet_1", elem_b="front_pivot_collar",
        reason="Front pivot rivet head clamps the outside of the scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_rivet_0", elem_b="rear_pivot_collar",
        reason="Rear pivot rivet head clamps the outside of the scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_rivet_1", elem_b="rear_pivot_collar",
        reason="Rear pivot rivet head clamps the outside of the scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_rivet_0", elem_b="front_moving_diagonal",
        reason="Front pivot rivet passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, brace_0,
        elem_a="front_pivot_rivet_1", elem_b="front_moving_diagonal",
        reason="Front pivot rivet passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_rivet_0", elem_b="rear_moving_diagonal",
        reason="Rear pivot rivet passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, brace_1,
        elem_a="rear_pivot_rivet_1", elem_b="rear_moving_diagonal",
        reason="Rear pivot rivet passes through the moving brace tube at the scissor joint.",
    )

    # ---------- Provenance ----------

    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # ---------- Structural topology change ----------

    # Confirm the old quad X-brace front cross member no longer exists
    try:
        object_model.get_part("front_cross_brace")
        front_cross_gone = False
    except Exception:
        front_cross_gone = True
    ctx.check(
        "front_cross_brace removed in director's chair redesign",
        front_cross_gone,
        details="The quad X-brace front cross member was dropped; director's chair uses side X-scissors only.",
    )

    # Director's chair rigid rectangular side frames
    ctx.check(
        "left side frame has front leg and top rail",
        frame.get_visual("left_front_leg") is not None
        and frame.get_visual("left_top_rail") is not None
        and frame.get_visual("left_rear_post") is not None,
        details="Left rigid rectangular side frame must have front leg, rear post, and top arm rail.",
    )
    ctx.check(
        "right side frame has front leg and top rail",
        frame.get_visual("right_front_leg") is not None
        and frame.get_visual("right_top_rail") is not None
        and frame.get_visual("right_rear_post") is not None,
        details="Right rigid rectangular side frame must have front leg, rear post, and top arm rail.",
    )

    # Flat taut canvas panels
    ctx.check(
        "flat taut seat canvas panel",
        frame.get_visual("seat_gray_center") is not None,
        details="Director's chair must have a flat taut seat canvas stretched between the side rails.",
    )
    ctx.check(
        "flat taut back canvas panel",
        frame.get_visual("back_gray_center") is not None,
        details="Director's chair must have a flat taut back canvas panel between the rear posts.",
    )

    # Hardwood arm boards
    ctx.check(
        "hardwood armrest boards on top rails",
        frame.get_visual("arm_rest_top_0") is not None
        and frame.get_visual("arm_rest_top_1") is not None,
        details="Director's chair armrests are straight rigid hardwood boards on the top side rails.",
    )

    # ---------- Pivot retention ----------

    ctx.expect_overlap(
        frame, brace_0, axes="xz",
        min_overlap=0.015,
        elem_a="front_pivot_pin", elem_b="front_pivot_collar",
        name="front X-brace collar retained on pivot pin",
    )
    ctx.expect_overlap(
        frame, brace_1, axes="xz",
        min_overlap=0.015,
        elem_a="rear_pivot_pin", elem_b="rear_pivot_collar",
        name="rear X-brace collar retained on pivot pin",
    )
    ctx.expect_overlap(
        frame, brace_0, axes="yz",
        min_overlap=0.008,
        elem_a="front_pivot_rivet_0", elem_b="front_pivot_collar",
        name="front rivet clamps pivot collar",
    )
    ctx.expect_overlap(
        frame, brace_1, axes="yz",
        min_overlap=0.008,
        elem_a="rear_pivot_rivet_0", elem_b="rear_pivot_collar",
        name="rear rivet clamps pivot collar",
    )

    # ---------- Joint configuration ----------

    ctx.check(
        "X-brace folding joints use synchronized Y-axis for sideways folding",
        tuple(joint_0.axis) == (0.0, 1.0, 0.0)
        and tuple(joint_1.axis) == (0.0, 1.0, 0.0)
        and joint_0.motion_limits.lower == 0.0
        and joint_1.motion_limits.lower == 0.0
        and joint_0.motion_limits.upper > 0.5
        and joint_1.motion_limits.upper > 0.5,
        details=f"front_axis={joint_0.axis} rear_axis={joint_1.axis}",
    )

    # ---------- Folding behaviour ----------

    closed_aabb_0 = ctx.part_world_aabb(brace_0)
    closed_aabb_1 = ctx.part_world_aabb(brace_1)

    # Get the brace end positions at rest (the X-brace endpoints that narrow)
    end0_rest = ctx.part_element_world_aabb(brace_0, elem="front_brace_end_0")
    end1_rest = ctx.part_element_world_aabb(brace_1, elem="rear_brace_end_0")

    with ctx.pose({joint_0: 0.50, joint_1: 0.50}):
        folded_aabb_0 = ctx.part_world_aabb(brace_0)
        folded_aabb_1 = ctx.part_world_aabb(brace_1)

        end0_folded = ctx.part_element_world_aabb(brace_0, elem="front_brace_end_0")
        end1_folded = ctx.part_element_world_aabb(brace_1, elem="rear_brace_end_0")

        ctx.expect_overlap(
            frame, brace_0, axes="xz",
            min_overlap=0.010,
            elem_a="front_pivot_pin", elem_b="front_pivot_collar",
            name="front pivot still captured while folding",
        )
        ctx.expect_overlap(
            frame, brace_1, axes="xz",
            min_overlap=0.010,
            elem_a="rear_pivot_pin", elem_b="rear_pivot_collar",
            name="rear pivot still captured while folding",
        )

    # The X-brace endpoints narrow (director's chair folds flat sideways)
    # Check that the outer endpoint's X-distance from center decreases
    if end0_rest is not None and end0_folded is not None:
        rest_span = max(abs(end0_rest[0][0]), abs(end0_rest[1][0]))
        folded_span = max(abs(end0_folded[0][0]), abs(end0_folded[1][0]))
        ctx.check(
            "front X-brace narrows when folding (director's chair folds flat sideways)",
            folded_span < rest_span - 0.02,
            details=f"rest_span={rest_span:.4f} folded_span={folded_span:.4f}",
        )
    if end1_rest is not None and end1_folded is not None:
        rest_span = max(abs(end1_rest[0][0]), abs(end1_rest[1][0]))
        folded_span = max(abs(end1_folded[0][0]), abs(end1_folded[1][0]))
        ctx.check(
            "rear X-brace narrows when folding (director's chair folds flat sideways)",
            folded_span < rest_span - 0.02,
            details=f"rest_span={rest_span:.4f} folded_span={folded_span:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
