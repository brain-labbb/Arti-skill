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

# ---------------------------------------------------------------------------
# Multiplicity parameter: number of seat bays side-by-side.
#   N=1 → single camp chair (original)
#   N=2 → loveseat / double bench (default here)
#   N=3 → triple camp bench
# ---------------------------------------------------------------------------
SEAT_CELL_COUNT = 2

# Companion color variation (⑥ two-tone bench colorway, no part-tree change).
# "charcoal" keeps the original palette; "red" or "blue" swap the seat/back accent.
COLOR_SCHEME = "charcoal"


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
        name="folding_camping_loveseat",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": (
                f"Double folding camping loveseat ({SEAT_CELL_COUNT} seat bays) "
                "based on the attached reference image, widened along the left-right axis."
            ),
            "seat_cell_count": SEAT_CELL_COUNT,
            "color_scheme": COLOR_SCHEME,
        },
    )

    # ---- materials --------------------------------------------------------
    black_fabric = model.material("black_oxford_fabric", rgba=(0.005, 0.005, 0.006, 1.0))
    # Accent fabric changes with COLOR_SCHEME companion variation.
    if COLOR_SCHEME == "red":
        accent_rgba = (0.58, 0.09, 0.09, 1.0)
    elif COLOR_SCHEME == "blue":
        accent_rgba = (0.09, 0.18, 0.52, 1.0)
    else:
        accent_rgba = (0.27, 0.29, 0.27, 1.0)
    gray_fabric = model.material("charcoal_gray_fabric", rgba=accent_rgba)
    orange_trim = model.material("orange_piping", rgba=(1.0, 0.32, 0.04, 1.0))
    metal = model.material("speckled_dark_tubular_steel", rgba=(0.43, 0.43, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet_heads", rgba=(0.86, 0.88, 0.86, 1.0))
    white = model.material("white_logo_and_stitches", rgba=(0.92, 0.92, 0.88, 1.0))
    mesh_mat = model.material("black_mesh_pocket", rgba=(0.01, 0.01, 0.01, 0.62))

    # ---- global geometry parameters ---------------------------------------
    seat_pitch = 0.86  # width of one seat bay (same as original single chair)
    half_width = SEAT_CELL_COUNT * seat_pitch / 2.0
    left_x = -half_width  # outer left leg line
    right_x = half_width  # outer right leg line
    total_width = SEAT_CELL_COUNT * seat_pitch

    front_y = -0.36
    rear_y = 0.30
    seat_z = 0.48
    arm_z_front = 0.66
    arm_z_rear = 0.73
    tube_r = 0.014
    front_pivot_y = front_y - 0.080

    frame = model.part("chair_frame")

    # ---- helper: bay center x for seat cell i ----------------------------
    def bay_cx(i: int) -> float:
        return left_x + seat_pitch * 0.5 + i * seat_pitch

    # ---- leg x positions (outer + dividers) ------------------------------
    leg_xs = [left_x + i * seat_pitch for i in range(SEAT_CELL_COUNT + 1)]

    # 1. Feet and upright tubular legs at every leg position (loop) ---------
    for idx, x in enumerate(leg_xs):
        _add_box(frame, f"front_foot_{idx}", (0.115, 0.082, 0.026), (x, front_y, 0.013), black_plastic)
        _add_ball(frame, f"front_foot_rivet_{idx}", (x, front_y, 0.031), 0.010, silver)
        _add_box(frame, f"rear_foot_{idx}", (0.115, 0.082, 0.026), (x, rear_y, 0.013), black_plastic)
        _add_ball(frame, f"rear_foot_rivet_{idx}", (x, rear_y, 0.031), 0.010, silver)
        _add_tube(frame, f"front_leg_{idx}", (x, front_y, 0.025), (x, front_y, arm_z_front), tube_r, metal)
        _add_tube(frame, f"rear_back_post_{idx}", (x, rear_y, 0.025), (x, rear_y + 0.07, 1.12), tube_r, metal)

    # 2. Full-width horizontal rails ----------------------------------------
    _add_tube(frame, "front_seat_rail", (left_x, front_y, seat_z), (right_x, front_y, seat_z), tube_r, metal)
    _add_tube(frame, "rear_seat_rail", (left_x, rear_y, seat_z + 0.05), (right_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(frame, "front_fixed_cross", (left_x, front_y + 0.018, seat_z), (right_x, front_y + 0.018, 0.070), tube_r, metal)
    _add_tube(frame, "rear_lower_cross", (left_x, rear_y, 0.10), (right_x, rear_y, 0.10), tube_r, metal)
    _add_tube(frame, "front_lower_cross", (left_x, front_y, 0.10), (right_x, front_y, 0.10), tube_r, metal)
    _add_tube(frame, "top_back_rail", (left_x, rear_y + 0.07, 1.12), (right_x, rear_y + 0.07, 1.12), tube_r, metal)

    # 3. Side seat rails: outer edges only ----------------------------------
    _add_tube(frame, "side_seat_rail_0", (left_x, front_y, seat_z), (left_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(frame, "side_seat_rail_1", (right_x, front_y, seat_z), (right_x, rear_y, seat_z + 0.05), tube_r, metal)

    # 4. Side fixed crosses: outer edges only --------------------------------
    _add_tube(frame, "side_fixed_cross_0", (left_x + 0.018, front_y, seat_z), (left_x + 0.018, rear_y, 0.075), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_1", (right_x - 0.018, front_y, seat_z), (right_x - 0.018, rear_y, 0.075), tube_r, metal)

    # 5. Arm sleeve cores: outer edges only ---------------------------------
    _add_tube(frame, "arm_sleeve_core_0", (left_x, front_y + 0.025, 0.682), (left_x, rear_y + 0.015, 0.704), 0.010, black_fabric)
    _add_tube(frame, "arm_sleeve_core_1", (right_x, front_y + 0.025, 0.682), (right_x, rear_y + 0.015, 0.704), 0.010, black_fabric)

    # 6. Per-bay front pivot pins, supports, and rivets (loop) --------------
    for i in range(SEAT_CELL_COUNT):
        cx = bay_cx(i)
        _add_tube(frame, f"front_pivot_pin_{i}", (cx, front_pivot_y - 0.032, 0.300), (cx, front_pivot_y + 0.032, 0.300), 0.018, black_plastic)
        _add_tube(frame, f"front_pivot_support_{i}", (cx, front_pivot_y + 0.026, 0.300), (cx, front_y, seat_z), 0.010, black_plastic)
        _add_ball(frame, f"front_pivot_rivet_{i}", (cx, front_pivot_y - 0.032, 0.300), 0.010, silver)

    # 7. Side pivot infrastructure: outer edges only -------------------------
    _add_tube(frame, "side_pivot_pin_0", (left_x - 0.038, -0.020, 0.325), (left_x + 0.026, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_pin_1", (right_x - 0.026, -0.020, 0.325), (right_x + 0.038, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_support_0", (left_x + 0.026, -0.020, 0.325), (left_x + 0.018, -0.020, 0.272), 0.007, black_plastic)
    _add_tube(frame, "side_pivot_support_1", (right_x - 0.026, -0.020, 0.325), (right_x - 0.018, -0.020, 0.272), 0.007, black_plastic)
    _add_ball(frame, "side_pivot_rivet_0", (left_x - 0.047, -0.020, 0.325), 0.010, silver)
    _add_ball(frame, "side_pivot_rivet_1", (right_x + 0.047, -0.020, 0.325), 0.010, silver)

    # 8. Per-bay seat cells: sling + piping (loop) --------------------------
    for i in range(SEAT_CELL_COUNT):
        cx = bay_cx(i)
        _add_box(frame, f"seat_black_left_{i}", (0.17, 0.68, 0.020), (cx - 0.130, -0.035, seat_z + 0.012), black_fabric)
        _add_box(frame, f"seat_black_right_{i}", (0.17, 0.68, 0.020), (cx + 0.130, -0.035, seat_z + 0.012), black_fabric)
        _add_box(frame, f"seat_gray_center_{i}", (0.42, 0.68, 0.018), (cx, -0.035, seat_z + 0.014), gray_fabric)
        _add_box(frame, f"seat_piping_0_{i}", (0.012, 0.650, 0.010), (cx - 0.205, -0.035, seat_z + 0.031), orange_trim)
        _add_box(frame, f"seat_piping_1_{i}", (0.012, 0.650, 0.010), (cx + 0.205, -0.035, seat_z + 0.031), orange_trim)

    # 9. Per-bay back panels: center + sides + piping + logo (loop) ---------
    back_roll = -0.15
    for i in range(SEAT_CELL_COUNT):
        cx = bay_cx(i)
        _add_box(frame, f"back_gray_center_{i}", (0.42, 0.028, 0.690), (cx, 0.285, 0.825), gray_fabric, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_black_side_0_{i}", (0.18, 0.030, 0.700), (cx - 0.305, 0.285, 0.825), black_fabric, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_black_side_1_{i}", (0.18, 0.030, 0.700), (cx + 0.305, 0.285, 0.825), black_fabric, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_piping_0_{i}", (0.012, 0.026, 0.650), (cx - 0.210, 0.246, 0.850), orange_trim, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_piping_1_{i}", (0.012, 0.026, 0.650), (cx + 0.210, 0.246, 0.850), orange_trim, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_logo_mark_{i}", (0.090, 0.045, 0.035), (cx, 0.285, 1.040), white, rpy=(back_roll, 0.0, 0.0))
        _add_box(frame, f"back_logo_text_{i}", (0.160, 0.045, 0.022), (cx, 0.285, 0.995), white, rpy=(back_roll, 0.0, 0.0))

    # 10. Seat divider strips between adjacent bays (loop) ------------------
    for i in range(SEAT_CELL_COUNT - 1):
        div_x = left_x + (i + 1) * seat_pitch
        _add_box(frame, f"seat_divider_strip_{i}", (0.06, 0.68, 0.022), (div_x, -0.035, seat_z + 0.013), black_fabric)
        _add_box(frame, f"back_divider_strip_{i}", (0.06, 0.030, 0.690), (div_x, 0.285, 0.825), black_fabric, rpy=(back_roll, 0.0, 0.0))

    # 11. Full-width shared seat/back trim ----------------------------------
    _add_box(frame, "seat_front_roll", (total_width + 0.04, 0.040, 0.035), (0.0, front_y - 0.015, seat_z + 0.022), black_fabric)
    _add_box(frame, "seat_rear_lap", (total_width - 0.04, 0.035, 0.030), (0.0, rear_y + 0.010, seat_z + 0.050), black_fabric)
    _add_box(frame, "front_edge_stitch", (total_width - 0.10, 0.008, 0.010), (0.0, front_y - 0.038, seat_z + 0.044), white)
    _add_box(frame, "back_top_pillow", (total_width - 0.10, 0.045, 0.110), (0.0, 0.340, 1.150), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(frame, "back_lower_band", (total_width - 0.14, 0.033, 0.090), (0.0, 0.232, 0.565), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(frame, "back_top_stitch", (total_width - 0.14, 0.030, 0.018), (0.0, 0.336, 1.155), white, rpy=(back_roll, 0.0, 0.0))

    # 12. Arm rests: outer ends only ----------------------------------------
    for x, side, outer_sign in ((left_x, "0", -1.0), (right_x, "1", 1.0)):
        _add_box(frame, f"arm_rest_top_{side}", (0.165, 0.585, 0.022), (x, -0.040, 0.703), black_fabric)
        _add_box(frame, f"arm_rest_outer_skirt_{side}", (0.036, 0.560, 0.110), (x + outer_sign * 0.072, -0.030, 0.660), black_fabric)
        _add_box(frame, f"arm_rest_front_cuff_{side}", (0.170, 0.050, 0.070), (x, front_y + 0.018, 0.676), black_fabric)
        _add_box(frame, f"arm_rest_rear_cuff_{side}", (0.170, 0.055, 0.075), (x, rear_y + 0.025, 0.724), black_fabric)
        _add_box(frame, f"arm_rest_outer_stitch_{side}", (0.010, 0.545, 0.010), (x + outer_sign * 0.086, -0.035, 0.716), white)
        _add_box(frame, f"arm_rest_inner_stitch_{side}", (0.010, 0.520, 0.010), (x - outer_sign * 0.054, -0.030, 0.715), white)
        _add_box(frame, f"arm_rest_front_bar_tack_{side}", (0.145, 0.010, 0.012), (x, front_y + 0.046, 0.713), orange_trim)
        _add_box(frame, f"arm_rest_rear_bar_tack_{side}", (0.145, 0.010, 0.012), (x, rear_y - 0.004, 0.742), orange_trim)

    # 13. Cup holder and side pocket on outer left arm ----------------------
    cup_ring = mesh_from_geometry(
        TorusGeometry(radius=0.055, tube=0.006, radial_segments=20, tubular_segments=36),
        "cup_holder_ring",
    )
    frame.visual(cup_ring, origin=Origin(xyz=(left_x - 0.005, -0.245, 0.724)), material=black_plastic, name="cup_holder_ring")
    frame.visual(
        Cylinder(radius=0.050, length=0.100),
        origin=Origin(xyz=(left_x, -0.245, 0.655)),
        material=mesh_mat,
        name="cup_holder_sleeve",
    )
    _add_box(frame, "side_pocket_front", (0.160, 0.018, 0.230), (left_x - 0.012, -0.242, 0.535), mesh_mat)
    _add_box(frame, "side_pocket_outer", (0.020, 0.180, 0.230), (left_x - 0.082, -0.160, 0.535), mesh_mat)
    _add_box(frame, "side_pocket_label", (0.045, 0.014, 0.072), (left_x - 0.012, -0.252, 0.565), white)

    # ======================================================================
    # Articulated brace children — one front scissor brace per bay
    # ======================================================================
    front_braces = []
    front_joints = []
    for i in range(SEAT_CELL_COUNT):
        cx = bay_cx(i)
        brace = model.part(f"front_cross_brace_{i}")
        _add_tube(brace, f"front_moving_tube_{i}", (-0.390, -0.065, -0.235), (0.390, -0.065, 0.235), tube_r, metal)
        _add_tube(brace, f"front_pivot_collar_{i}", (0.0, -0.054, 0.0), (0.0, 0.018, 0.0), 0.020, black_plastic)
        _add_ball(brace, f"front_brace_end_0_{i}", (-0.390, -0.065, -0.235), 0.018, black_plastic)
        _add_ball(brace, f"front_brace_end_1_{i}", (0.390, -0.065, 0.235), 0.018, black_plastic)
        front_braces.append(brace)

        joint = model.articulation(
            f"frame_to_front_cross_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=brace,
            origin=Origin(xyz=(cx, front_pivot_y, 0.300)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=0.65),
        )
        front_joints.append(joint)

    # Side cross braces: outer edges only (same as parent) -----------------
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
    side0 = object_model.get_part("side_cross_brace_0")
    side1 = object_model.get_part("side_cross_brace_1")
    side_joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    side_joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # Collect per-bay front brace parts and joints.
    front_braces = [object_model.get_part(f"front_cross_brace_{i}") for i in range(SEAT_CELL_COUNT)]
    front_joints = [object_model.get_articulation(f"frame_to_front_cross_{i}") for i in range(SEAT_CELL_COUNT)]

    # ---- overlap allowances: front pivot pins captured by collars ---------
    for i in range(SEAT_CELL_COUNT):
        ctx.allow_overlap(
            frame,
            front_braces[i],
            elem_a=f"front_pivot_pin_{i}",
            elem_b=f"front_pivot_collar_{i}",
            reason=f"Bay {i}: the moving scissor brace collar is intentionally captured around the fixed front pivot pin.",
        )
        ctx.allow_overlap(
            frame,
            front_braces[i],
            elem_a=f"front_pivot_rivet_{i}",
            elem_b=f"front_pivot_collar_{i}",
            reason=f"Bay {i}: the front rivet head intentionally captures the rotating scissor collar on the pivot.",
        )

    # Side pivot allowances (outer only, same as parent)
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
        frame, side0,
        elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, side1,
        elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )

    # ---- provenance metadata ----------------------------------------------
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # ---- loveseat multiplicity check: N seat cells exist ------------------
    seat_cell_names = [f"seat_gray_center_{i}" for i in range(SEAT_CELL_COUNT)]
    back_cell_names = [f"back_gray_center_{i}" for i in range(SEAT_CELL_COUNT)]
    all_seat_present = all(
        frame.get_visual(n) is not None for n in seat_cell_names
    )
    all_back_present = all(
        frame.get_visual(n) is not None for n in back_cell_names
    )
    ctx.check(
        f"loveseat has {SEAT_CELL_COUNT} seat cells with indexed seat/back panels",
        all_seat_present and all_back_present,
        details=f"seat_cells={seat_cell_names}, back_cells={back_cell_names}",
    )

    # ---- loveseat width: wider than a single chair -----------------------
    seat_pitch = 0.86
    expected_min_width = SEAT_CELL_COUNT * seat_pitch - 0.10
    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        frame_width_x = frame_aabb[1][0] - frame_aabb[0][0]
        ctx.check(
            f"loveseat frame is at least {expected_min_width:.2f}m wide ({SEAT_CELL_COUNT} bays)",
            frame_width_x >= expected_min_width,
            details=f"measured_width={frame_width_x:.3f}m, expected_min={expected_min_width:.2f}m",
        )

    # ---- front pivot retention per bay ------------------------------------
    for i in range(SEAT_CELL_COUNT):
        ctx.expect_overlap(
            frame, front_braces[i],
            axes="yz", min_overlap=0.020,
            elem_a=f"front_pivot_pin_{i}", elem_b=f"front_pivot_collar_{i}",
            name=f"bay {i} front brace collar retained on pivot",
        )
        ctx.expect_overlap(
            frame, front_braces[i],
            axes="xz", min_overlap=0.010,
            elem_a=f"front_pivot_rivet_{i}", elem_b=f"front_pivot_collar_{i}",
            name=f"bay {i} front rivet head captures pivot collar",
        )
        ctx.expect_gap(
            frame, front_braces[i],
            axis="z", max_penetration=0.05,
            elem_a=f"front_pivot_pin_{i}", elem_b=f"front_pivot_collar_{i}",
            name=f"bay {i} front pivot overlap remains localized",
        )

    # ---- side brace retention --------------------------------------------
    ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left side brace collar retained on pivot")
    ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right side brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0", name="left rivet head clamps pivot collar")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1", name="right rivet head clamps pivot collar")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0", name="left rivet passes through moving brace")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1", name="right rivet passes through moving brace")

    # ---- mirrored side folding pivots -------------------------------------
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

    # ---- folding pose: all braces change angle ----------------------------
    closed_front_aabbs = [ctx.part_world_aabb(b) for b in front_braces]
    side0_closed = ctx.part_world_aabb(side0)
    side1_closed = ctx.part_world_aabb(side1)

    pose_dict = {}
    for j in front_joints:
        pose_dict[j] = 0.45
    pose_dict[side_joint_0] = 0.45
    pose_dict[side_joint_1] = 0.45

    with ctx.pose(pose_dict):
        folded_front_aabbs = [ctx.part_world_aabb(b) for b in front_braces]
        side0_folded = ctx.part_world_aabb(side0)
        side1_folded = ctx.part_world_aabb(side1)

        # Front pivot still captured while folding
        for i in range(SEAT_CELL_COUNT):
            ctx.expect_overlap(
                frame, front_braces[i],
                axes="yz", min_overlap=0.015,
                elem_a=f"front_pivot_pin_{i}", elem_b=f"front_pivot_collar_{i}",
                name=f"bay {i} front pivot still captured while folding",
            )
        ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left pivot still captured while folding")
        ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.015, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right pivot still captured while folding")

    # Each bay's front brace changes angle during folding
    for i in range(SEAT_CELL_COUNT):
        ca = closed_front_aabbs[i]
        fa = folded_front_aabbs[i]
        ctx.check(
            f"bay {i} front cross brace changes angle when folding",
            ca is not None and fa is not None
            and abs((fa[1][2] - fa[0][2]) - (ca[1][2] - ca[0][2])) > 0.025,
            details=f"rest={ca}, folded={fa}",
        )

    ctx.check(
        "side braces fold as a mirrored pair",
        side0_closed is not None
        and side1_closed is not None
        and side0_folded is not None
        and side1_folded is not None
        and abs((side0_folded[1][1] - side0_folded[0][1]) - (side0_closed[1][1] - side0_closed[0][1])) > 0.020
        and abs((side1_folded[1][1] - side1_folded[0][1]) - (side1_closed[1][1] - side1_closed[0][1])) > 0.020,
        details=f"left_rest={side0_closed}, left_folded={side0_folded}, right_rest={side1_closed}, right_folded={side1_folded}",
    )

    # ---- loveseat-specific: front_cross_brace_0 and front_cross_brace_1 exist
    ctx.check(
        "loveseat has one front cross brace per seat bay",
        len(front_braces) == SEAT_CELL_COUNT
        and all(b is not None for b in front_braces),
        details=f"front_brace_parts={[f'front_cross_brace_{i}' for i in range(SEAT_CELL_COUNT)]}",
    )

    return ctx.report()


object_model = build_object_model()
