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
    TorusGeometry,
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
        name="folding_camping_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Reclining folding camping chair with adjustable backrest based on the reference image.",
        },
    )

    black_fabric = model.material("black_oxford_fabric", rgba=(0.005, 0.005, 0.006, 1.0))
    gray_fabric = model.material("charcoal_gray_fabric", rgba=(0.27, 0.29, 0.27, 1.0))
    orange_trim = model.material("orange_piping", rgba=(1.0, 0.32, 0.04, 1.0))
    metal = model.material("speckled_dark_tubular_steel", rgba=(0.43, 0.43, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet_heads", rgba=(0.86, 0.88, 0.86, 1.0))
    white = model.material("white_logo_and_stitches", rgba=(0.92, 0.92, 0.88, 1.0))
    mesh_mat = model.material("black_mesh_pocket", rgba=(0.01, 0.01, 0.01, 0.62))

    frame = model.part("chair_frame")

    # Main foot locations and fixed upper frame points.  The chair faces -Y;
    # the high back and rear uprights sit toward +Y.
    front_y = -0.36
    rear_y = 0.30
    left_x = -0.43
    right_x = 0.43
    seat_z = 0.48
    arm_z_front = 0.66
    arm_z_rear = 0.73
    tube_r = 0.014

    # Recline hinge line: at the rear seat rail height where the back meets the seat.
    hinge_y = rear_y
    hinge_z = seat_z + 0.05  # 0.53

    # Four feet and upright tubular legs.
    for x, y, suffix in (
        (left_x, front_y, "front_0"),
        (right_x, front_y, "front_1"),
        (left_x, rear_y, "rear_0"),
        (right_x, rear_y, "rear_1"),
    ):
        _add_box(frame, f"{suffix}_foot", (0.115, 0.082, 0.026), (x, y, 0.013), black_plastic)
        _add_ball(frame, f"{suffix}_foot_rivet", (x, y, 0.031), 0.010, silver)

    _add_tube(frame, "front_leg_0", (left_x, front_y, 0.025), (left_x, front_y, arm_z_front), tube_r, metal)
    _add_tube(frame, "front_leg_1", (right_x, front_y, 0.025), (right_x, front_y, arm_z_front), tube_r, metal)

    # Lower rear stub legs: fixed to the frame, from feet up to the recline hinge.
    # The upper back posts (on back_recliner) continue from the hinge upward.
    _add_tube(frame, "rear_lower_leg_0", (left_x, rear_y, 0.025), (left_x, rear_y + 0.032, hinge_z), tube_r, metal)
    _add_tube(frame, "rear_lower_leg_1", (right_x, rear_y, 0.025), (right_x, rear_y + 0.032, hinge_z), tube_r, metal)

    # Fixed perimeter rails and one member of each X brace, like the reference camp chair.
    _add_tube(frame, "front_seat_rail", (left_x, front_y, seat_z), (right_x, front_y, seat_z), tube_r, metal)
    _add_tube(frame, "rear_seat_rail", (left_x, rear_y, seat_z + 0.05), (right_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(frame, "side_seat_rail_0", (left_x, front_y, seat_z), (left_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(frame, "side_seat_rail_1", (right_x, front_y, seat_z), (right_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(frame, "front_fixed_cross", (left_x, front_y + 0.018, seat_z), (right_x, front_y + 0.018, 0.070), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_0", (left_x + 0.018, front_y, seat_z), (left_x + 0.018, rear_y, 0.075), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_1", (right_x - 0.018, front_y, seat_z), (right_x - 0.018, rear_y, 0.075), tube_r, metal)
    _add_tube(frame, "rear_lower_cross", (left_x, rear_y, 0.10), (right_x, rear_y, 0.10), tube_r, metal)
    _add_tube(frame, "front_lower_cross", (left_x, front_y, 0.10), (right_x, front_y, 0.10), tube_r, metal)

    # Arm support cores are wrapped by the sewn fabric sleeves below; keep them
    # visually black and tucked inside the cloth so no grey tube protrudes from
    # the arm rests.
    _add_tube(frame, "arm_sleeve_core_0", (left_x, front_y + 0.025, 0.682), (left_x, rear_y - 0.040, 0.696), 0.010, black_fabric)
    _add_tube(frame, "arm_sleeve_core_1", (right_x, front_y + 0.025, 0.682), (right_x, rear_y - 0.040, 0.696), 0.010, black_fabric)

    # Recline hinge pin: fixed to the frame at the hinge line, captured by the
    # back_recliner pivot collar.
    _add_tube(frame, "recline_hinge_pin", (left_x + 0.05, hinge_y, hinge_z), (right_x - 0.05, hinge_y, hinge_z), 0.018, black_plastic)
    for x, name in ((left_x + 0.040, "recline_hinge_rivet_0"), (right_x - 0.040, "recline_hinge_rivet_1")):
        _add_ball(frame, name, (x, hinge_y, hinge_z), 0.010, silver)

    # Pivot pins and clevis-like brackets for the articulated folding braces.
    front_pivot_y = front_y - 0.080
    _add_tube(frame, "front_pivot_pin", (-0.002, front_pivot_y - 0.032, 0.300), (-0.002, front_pivot_y + 0.032, 0.300), 0.018, black_plastic)
    _add_tube(frame, "front_pivot_support", (0.0, front_pivot_y + 0.026, 0.300), (0.0, front_y, seat_z), 0.010, black_plastic)
    _add_tube(frame, "side_pivot_pin_0", (left_x - 0.038, -0.020, 0.325), (left_x + 0.026, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_pin_1", (right_x - 0.026, -0.020, 0.325), (right_x + 0.038, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_support_0", (left_x + 0.026, -0.020, 0.325), (left_x + 0.018, -0.020, 0.272), 0.007, black_plastic)
    _add_tube(frame, "side_pivot_support_1", (right_x - 0.026, -0.020, 0.325), (right_x - 0.018, -0.020, 0.272), 0.007, black_plastic)
    for xyz, name in (
        ((-0.002, front_pivot_y - 0.032, 0.300), "front_pivot_rivet"),
        ((left_x - 0.047, -0.020, 0.325), "side_pivot_rivet_0"),
        ((right_x + 0.047, -0.020, 0.325), "side_pivot_rivet_1"),
    ):
        _add_ball(frame, name, xyz, 0.010, silver)

    # Sling seat: black outside fabric, grey inset center, orange seams and raised black edging.
    _add_box(frame, "seat_black_left", (0.17, 0.68, 0.020), (-0.300, -0.035, seat_z + 0.012), black_fabric)
    _add_box(frame, "seat_black_right", (0.17, 0.68, 0.020), (0.300, -0.035, seat_z + 0.012), black_fabric)
    _add_box(frame, "seat_gray_center", (0.42, 0.68, 0.018), (0.0, -0.035, seat_z + 0.014), gray_fabric)
    _add_box(frame, "seat_front_roll", (0.90, 0.040, 0.035), (0.0, front_y - 0.015, seat_z + 0.022), black_fabric)
    _add_box(frame, "seat_rear_lap", (0.84, 0.035, 0.030), (0.0, rear_y + 0.010, seat_z + 0.050), black_fabric)
    _add_box(frame, "seat_piping_0", (0.012, 0.650, 0.010), (-0.205, -0.035, seat_z + 0.031), orange_trim)
    _add_box(frame, "seat_piping_1", (0.012, 0.650, 0.010), (0.205, -0.035, seat_z + 0.031), orange_trim)
    _add_box(frame, "front_edge_stitch", (0.82, 0.008, 0.010), (0.0, front_y - 0.038, seat_z + 0.044), white)

    # Fabric arm rests, cup holder, and a hanging side pocket on the left arm.
    # The reference chair has soft fabric sleeves riding on the side rails, not
    # rigid boards stabbed through the back panel, so each arm gets a top sling,
    # front/rear sewn cuffs around the posts, and a hanging outer skirt.
    for x, side, outer_sign in ((left_x, "0", -1.0), (right_x, "1", 1.0)):
        _add_box(frame, f"arm_rest_top_{side}", (0.165, 0.585, 0.022), (x, -0.040, 0.703), black_fabric)
        _add_box(frame, f"arm_rest_outer_skirt_{side}", (0.036, 0.560, 0.110), (x + outer_sign * 0.072, -0.030, 0.660), black_fabric)
        _add_box(frame, f"arm_rest_front_cuff_{side}", (0.170, 0.050, 0.070), (x, front_y + 0.018, 0.676), black_fabric)
        _add_box(frame, f"arm_rest_rear_cuff_{side}", (0.170, 0.055, 0.075), (x, rear_y + 0.025, 0.724), black_fabric)
        _add_box(frame, f"arm_rest_outer_stitch_{side}", (0.010, 0.545, 0.010), (x + outer_sign * 0.086, -0.035, 0.716), white)
        _add_box(frame, f"arm_rest_inner_stitch_{side}", (0.010, 0.520, 0.010), (x - outer_sign * 0.054, -0.030, 0.715), white)
        _add_box(frame, f"arm_rest_front_bar_tack_{side}", (0.145, 0.010, 0.012), (x, front_y + 0.046, 0.713), orange_trim)
        _add_box(frame, f"arm_rest_rear_bar_tack_{side}", (0.145, 0.010, 0.012), (x, rear_y - 0.004, 0.742), orange_trim)
    cup_ring = mesh_from_geometry(TorusGeometry(radius=0.055, tube=0.006, radial_segments=20, tubular_segments=36), "cup_holder_ring")
    frame.visual(cup_ring, origin=Origin(xyz=(left_x - 0.005, -0.245, 0.724)), material=black_plastic, name="cup_holder_ring")
    frame.visual(Cylinder(radius=0.050, length=0.100), origin=Origin(xyz=(left_x, -0.245, 0.655)), material=mesh_mat, name="cup_holder_sleeve")
    _add_box(frame, "side_pocket_front", (0.160, 0.018, 0.230), (left_x - 0.012, -0.242, 0.535), mesh_mat)
    _add_box(frame, "side_pocket_outer", (0.020, 0.180, 0.230), (left_x - 0.082, -0.160, 0.535), mesh_mat)
    _add_box(frame, "side_pocket_label", (0.045, 0.014, 0.072), (left_x - 0.012, -0.252, 0.565), white)

    # ---- Reclining backrest child part ----
    # The back_recliner part frame sits at the hinge line (center of the recline
    # pivot). All back visuals are in local coordinates relative to this origin.
    back_recliner = model.part("back_recliner")

    # Upper rear posts (from hinge upward) and top rail, in local coords.
    # World hinge point: (0, hinge_y, hinge_z). Upper post goes to (x, rear_y+0.07, 1.12).
    # Start posts slightly above the hinge line to avoid overlap with side seat rails.
    upper_post_top_y_local = (rear_y + 0.07) - hinge_y  # 0.07
    upper_post_top_z_local = 1.12 - hinge_z  # 0.59
    post_base_z_local = 0.025  # start above hinge to clear side seat rails
    post_base_y_local = upper_post_top_y_local * (post_base_z_local / upper_post_top_z_local)
    _add_tube(back_recliner, "rear_back_post_0", (left_x, post_base_y_local, post_base_z_local), (left_x, upper_post_top_y_local, upper_post_top_z_local), tube_r, metal)
    _add_tube(back_recliner, "rear_back_post_1", (right_x, post_base_y_local, post_base_z_local), (right_x, upper_post_top_y_local, upper_post_top_z_local), tube_r, metal)
    _add_tube(back_recliner, "top_back_rail", (left_x, upper_post_top_y_local, upper_post_top_z_local), (right_x, upper_post_top_y_local, upper_post_top_z_local), tube_r, metal)

    # Recline pivot collar: captured around the frame hinge pin.
    # Must reach both rear_back_post bases so back_recliner forms one connected
    # geometry group.  Sits just behind the seat_rear_lap fabric (intentional
    # local overlap handled by allowance below).
    _add_tube(back_recliner, "recline_pivot_collar", (left_x, 0.0, 0.0), (right_x, 0.0, 0.0), 0.016, black_plastic)

    # Tall padded back panels, leaned backward.  Coordinates are local to the
    # hinge origin (subtract hinge_y from y, hinge_z from z).  The panels start
    # at the hinge (local z=0) and extend upward, not below into the seat.
    back_roll = -0.15
    back_panel_z_local = 0.35  # center of 0.700-tall panel, sitting on hinge
    back_panel_y_local = 0.010  # slightly behind the hinge axis
    _add_box(back_recliner, "back_gray_center", (0.42, 0.028, 0.690), (0.0, back_panel_y_local, back_panel_z_local), gray_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_black_side_0", (0.18, 0.030, 0.700), (-0.305, back_panel_y_local, back_panel_z_local), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_black_side_1", (0.18, 0.030, 0.700), (0.305, back_panel_y_local, back_panel_z_local), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_top_pillow", (0.82, 0.045, 0.110), (0.0, 0.055, 0.650), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_lower_band", (0.78, 0.033, 0.090), (0.0, -0.053, 0.035), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_piping_0", (0.012, 0.026, 0.650), (-0.210, -0.029, 0.320), orange_trim, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_piping_1", (0.012, 0.026, 0.650), (0.210, -0.029, 0.320), orange_trim, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_top_stitch", (0.78, 0.030, 0.018), (0.0, 0.051, 0.655), white, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_logo_mark", (0.090, 0.045, 0.035), (0.0, 0.010, 0.510), white, rpy=(back_roll, 0.0, 0.0))
    _add_box(back_recliner, "back_logo_text", (0.160, 0.045, 0.022), (0.0, 0.010, 0.465), white, rpy=(back_roll, 0.0, 0.0))

    # ---- Articulated brace children. Each has its local origin at the scissor pivot.
    front_brace = model.part("front_cross_brace")
    _add_tube(front_brace, "front_moving_tube", (-0.390, -0.065, -0.235), (0.390, -0.065, 0.235), tube_r, metal)
    _add_tube(front_brace, "front_pivot_collar", (0.0, -0.054, 0.0), (0.0, 0.018, 0.0), 0.020, black_plastic)
    _add_ball(front_brace, "front_brace_end_0", (-0.390, -0.065, -0.235), 0.018, black_plastic)
    _add_ball(front_brace, "front_brace_end_1", (0.390, -0.065, 0.235), 0.018, black_plastic)

    side_brace_0 = model.part("side_cross_brace_0")
    _add_tube(side_brace_0, "side_moving_tube_0", (-0.065, -0.335, -0.245), (-0.065, 0.330, 0.245), tube_r, metal)
    _add_tube(side_brace_0, "side_pivot_collar_0", (-0.052, 0.0, 0.0), (0.018, 0.0, 0.0), 0.020, black_plastic)
    _add_ball(side_brace_0, "side_brace_end_0a", (-0.065, -0.335, -0.245), 0.018, black_plastic)
    _add_ball(side_brace_0, "side_brace_end_0b", (-0.065, 0.330, 0.245), 0.018, black_plastic)

    side_brace_1 = model.part("side_cross_brace_1")
    _add_tube(side_brace_1, "side_moving_tube_1", (0.065, -0.335, -0.245), (0.065, 0.330, 0.245), tube_r, metal)
    _add_tube(side_brace_1, "side_pivot_collar_1", (-0.018, 0.0, 0.0), (0.052, 0.0, 0.0), 0.020, black_plastic)
    _add_ball(side_brace_1, "side_brace_end_1a", (0.065, -0.335, -0.245), 0.018, black_plastic)
    _add_ball(side_brace_1, "side_brace_end_1b", (0.065, 0.330, 0.245), 0.018, black_plastic)

    # ---- Articulations ----
    # Recline hinge: positive q tilts the backrest top toward +Y (backward recline).
    # axis=(-1,0,0) gives right-hand rotation from +Z toward +Y.
    model.articulation(
        "frame_to_recline_back",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_recliner,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=0.60),
    )

    model.articulation(
        "frame_to_front_cross",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_brace,
        origin=Origin(xyz=(0.0, front_pivot_y, 0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )
    model.articulation(
        "frame_to_side_cross_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_brace_0,
        origin=Origin(xyz=(left_x - 0.006, -0.020, 0.325)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )
    model.articulation(
        "frame_to_side_cross_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_brace_1,
        origin=Origin(xyz=(right_x + 0.006, -0.020, 0.325)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    back = object_model.get_part("back_recliner")
    front = object_model.get_part("front_cross_brace")
    side0 = object_model.get_part("side_cross_brace_0")
    side1 = object_model.get_part("side_cross_brace_1")
    recline_joint = object_model.get_articulation("frame_to_recline_back")
    front_joint = object_model.get_articulation("frame_to_front_cross")
    side_joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    side_joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # ---- Recline hinge overlap allowances ----
    ctx.allow_overlap(
        frame,
        back,
        elem_a="recline_hinge_pin",
        elem_b="recline_pivot_collar",
        reason="The backrest pivot collar is intentionally captured around the fixed recline hinge pin.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="recline_hinge_rivet_0",
        elem_b="recline_pivot_collar",
        reason="The left recline rivet head intentionally clamps the outside of the backrest pivot collar.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="recline_hinge_rivet_1",
        elem_b="recline_pivot_collar",
        reason="The right recline rivet head intentionally clamps the outside of the backrest pivot collar.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="rear_seat_rail",
        elem_b="recline_pivot_collar",
        reason="The backrest pivot collar wraps around the rear seat rail which serves as the recline hinge axis.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_cuff_0",
        elem_b="rear_back_post_0",
        reason="The left arm rest rear fabric cuff is sewn around the left rear back post.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_cuff_1",
        elem_b="rear_back_post_1",
        reason="The right arm rest rear fabric cuff is sewn around the right rear back post.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="seat_rear_lap",
        elem_b="recline_pivot_collar",
        reason="The rear seat fabric lap sits adjacent to the recline pivot collar at the hinge line; local overlap represents sewn fabric wrapping the hinge.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_top_0",
        elem_b="back_black_side_0",
        reason="The left arm rest fabric sits in front of the lower back panel; the back fabric extends behind the arm rest as in real sewn camp chair construction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_top_1",
        elem_b="back_black_side_1",
        reason="The right arm rest fabric sits in front of the lower back panel; the back fabric extends behind the arm rest as in real sewn camp chair construction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_cuff_0",
        elem_b="back_black_side_0",
        reason="The left arm rest rear cuff wraps around the back post and overlaps the lower back panel fabric at the sewn junction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_cuff_1",
        elem_b="back_black_side_1",
        reason="The right arm rest rear cuff wraps around the back post and overlaps the lower back panel fabric at the sewn junction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_bar_tack_0",
        elem_b="back_black_side_0",
        reason="The left arm rest rear reinforcement stitching overlaps the lower back panel at the recline junction; expected for sewn fabric construction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="arm_rest_rear_bar_tack_1",
        elem_b="back_black_side_1",
        reason="The right arm rest rear reinforcement stitching overlaps the lower back panel at the recline junction; expected for sewn fabric construction.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="side_seat_rail_0",
        elem_b="recline_pivot_collar",
        reason="The left side seat rail meets the recline pivot collar at the hinge junction; local overlap represents the structural connection.",
    )
    ctx.allow_overlap(
        frame,
        back,
        elem_a="side_seat_rail_1",
        elem_b="recline_pivot_collar",
        reason="The right side seat rail meets the recline pivot collar at the hinge junction; local overlap represents the structural connection.",
    )

    # ---- Scissor brace overlap allowances (preserved from parent) ----
    ctx.allow_overlap(
        frame,
        front,
        elem_a="front_pivot_pin",
        elem_b="front_pivot_collar",
        reason="The moving scissor brace collar is intentionally captured around the fixed front pivot pin.",
    )
    ctx.allow_overlap(
        frame,
        side0,
        elem_a="side_pivot_pin_0",
        elem_b="side_pivot_collar_0",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame,
        side1,
        elem_a="side_pivot_pin_1",
        elem_b="side_pivot_collar_1",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame,
        side0,
        elem_a="side_pivot_rivet_0",
        elem_b="side_pivot_collar_0",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame,
        side1,
        elem_a="side_pivot_rivet_1",
        elem_b="side_pivot_collar_1",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="front_pivot_rivet",
        elem_b="front_pivot_collar",
        reason="The front rivet head intentionally captures the rotating scissor collar on the pivot.",
    )
    ctx.allow_overlap(
        frame,
        side0,
        elem_a="side_pivot_rivet_0",
        elem_b="side_moving_tube_0",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame,
        side1,
        elem_a="side_pivot_rivet_1",
        elem_b="side_moving_tube_1",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )

    # ---- Structural / metadata checks ----
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # ---- Recline joint existence and motion limits ----
    ctx.check(
        "recline joint has detent-style upright-to-reclined range",
        recline_joint.motion_limits.lower == 0.0
        and 0.35 <= recline_joint.motion_limits.upper <= 0.80,
        details=f"lower={recline_joint.motion_limits.lower}, upper={recline_joint.motion_limits.upper}",
    )

    # ---- Recline pivot retained ----
    ctx.expect_overlap(frame, back, axes="yz", min_overlap=0.020, elem_a="recline_hinge_pin", elem_b="recline_pivot_collar", name="recline pivot collar retained on hinge pin")
    ctx.expect_overlap(frame, back, axes="yz", min_overlap=0.010, elem_a="recline_hinge_rivet_0", elem_b="recline_pivot_collar", name="left recline rivet clamps pivot collar")
    ctx.expect_overlap(frame, back, axes="yz", min_overlap=0.010, elem_a="recline_hinge_rivet_1", elem_b="recline_pivot_collar", name="right recline rivet clamps pivot collar")
    ctx.expect_gap(frame, back, axis="z", max_penetration=0.05, elem_a="recline_hinge_pin", elem_b="recline_pivot_collar", name="recline pivot overlap remains localized")

    # ---- Scissor brace retained checks (preserved from parent) ----
    ctx.expect_overlap(frame, front, axes="yz", min_overlap=0.020, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left side brace collar retained on pivot")
    ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right side brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0", name="left rivet head clamps pivot collar")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1", name="right rivet head clamps pivot collar")
    ctx.expect_overlap(frame, front, axes="xz", min_overlap=0.010, elem_a="front_pivot_rivet", elem_b="front_pivot_collar", name="front rivet head captures pivot collar")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0", name="left rivet passes through moving brace")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1", name="right rivet passes through moving brace")
    ctx.expect_gap(frame, front, axis="z", max_penetration=0.05, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front pivot overlap remains localized")

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

    # ---- Recline pose check: positive q tilts backrest top backward (+Y) ----
    back_upright_aabb = ctx.part_world_aabb(back)
    with ctx.pose({recline_joint: 0.45}):
        back_reclined_aabb = ctx.part_world_aabb(back)
        ctx.expect_overlap(frame, back, axes="yz", min_overlap=0.015, elem_a="recline_hinge_pin", elem_b="recline_pivot_collar", name="recline pivot still captured at reclined pose")

    ctx.check(
        "recline back tilts backward when positive q applied",
        back_upright_aabb is not None
        and back_reclined_aabb is not None
        and back_reclined_aabb[1][1] > back_upright_aabb[1][1] + 0.01,
        details=f"upright_max_y={back_upright_aabb[1][1]}, reclined_max_y={back_reclined_aabb[1][1]}",
    )

    # ---- Folding brace pose checks (preserved from parent) ----
    closed_aabb = ctx.part_world_aabb(front)
    side0_closed_aabb = ctx.part_world_aabb(side0)
    side1_closed_aabb = ctx.part_world_aabb(side1)
    with ctx.pose({front_joint: 0.45, side_joint_0: 0.45, side_joint_1: 0.45}):
        folded_aabb = ctx.part_world_aabb(front)
        side0_folded_aabb = ctx.part_world_aabb(side0)
        side1_folded_aabb = ctx.part_world_aabb(side1)
        ctx.expect_overlap(frame, front, axes="yz", min_overlap=0.015, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front pivot still captured while folding")
        ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left pivot still captured while folding")
        ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right pivot still captured while folding")

    ctx.check(
        "front cross brace changes angle",
        closed_aabb is not None
        and folded_aabb is not None
        and abs((folded_aabb[1][2] - folded_aabb[0][2]) - (closed_aabb[1][2] - closed_aabb[0][2])) > 0.025,
        details=f"rest={closed_aabb}, folded={folded_aabb}",
    )
    ctx.check(
        "side braces fold as a mirrored pair",
        side0_closed_aabb is not None
        and side1_closed_aabb is not None
        and side0_folded_aabb is not None
        and side1_folded_aabb is not None
        and abs((side0_folded_aabb[1][1] - side0_folded_aabb[0][1]) - (side0_closed_aabb[1][1] - side0_closed_aabb[0][1])) > 0.020
        and abs((side1_folded_aabb[1][1] - side1_folded_aabb[0][1]) - (side1_closed_aabb[1][1] - side1_closed_aabb[0][1])) > 0.020,
        details=f"left_rest={side0_closed_aabb}, left_folded={side0_folded_aabb}, right_rest={side1_closed_aabb}, right_folded={side1_folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
