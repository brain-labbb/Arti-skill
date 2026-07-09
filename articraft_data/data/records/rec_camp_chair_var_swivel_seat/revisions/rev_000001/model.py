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


def _spoke_start(hub_xyz, corner_xyz, offset: float):
    """Return a point along the line from hub to corner at the given distance from hub."""
    dx = corner_xyz[0] - hub_xyz[0]
    dy = corner_xyz[1] - hub_xyz[1]
    dz = corner_xyz[2] - hub_xyz[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return hub_xyz
    return (
        hub_xyz[0] + dx / length * offset,
        hub_xyz[1] + dy / length * offset,
        hub_xyz[2] + dz / length * offset,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="swivel_folding_camping_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "360-degree swivel folding camping chair with rotating seat assembly on a folding X-frame base. Dark-green swivel colorway.",
            "variant": "dark-green swivel colorway",
        },
    )

    # --- Materials (parent palette + dark-green swivel accent) ---
    black_fabric = model.material("black_oxford_fabric", rgba=(0.005, 0.005, 0.006, 1.0))
    gray_fabric = model.material("charcoal_gray_fabric", rgba=(0.27, 0.29, 0.27, 1.0))
    orange_trim = model.material("orange_piping", rgba=(1.0, 0.32, 0.04, 1.0))
    metal = model.material("speckled_dark_tubular_steel", rgba=(0.43, 0.43, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet_heads", rgba=(0.86, 0.88, 0.86, 1.0))
    white = model.material("white_logo_and_stitches", rgba=(0.92, 0.92, 0.88, 1.0))
    mesh_mat = model.material("black_mesh_pocket", rgba=(0.01, 0.01, 0.01, 0.62))
    dark_green = model.material("dark_green_swivel_accent", rgba=(0.10, 0.30, 0.12, 1.0))

    # --- Shared chair coordinates ---
    front_y = -0.36
    rear_y = 0.30
    left_x = -0.43
    right_x = 0.43
    seat_z = 0.48
    arm_z_front = 0.66
    tube_r = 0.014
    base_top_z = 0.45  # top of base leg stubs / X-frame plane

    # Swivel bearing centre sits just above the base X-frame plane.
    hub_xyz = (0.0, -0.03, base_top_z + 0.01)  # (0, -0.03, 0.46)
    spoke_clearance = 0.056  # radial offset so spokes clear the hub cylinder

    # =========================================================================
    # FOLDING BASE (chair_frame)
    # =========================================================================
    frame = model.part("chair_frame")

    # Four feet and foot rivets.
    for x, y, suffix in (
        (left_x, front_y, "front_0"),
        (right_x, front_y, "front_1"),
        (left_x, rear_y, "rear_0"),
        (right_x, rear_y, "rear_1"),
    ):
        _add_box(frame, f"{suffix}_foot", (0.115, 0.082, 0.026), (x, y, 0.013), black_plastic)
        _add_ball(frame, f"{suffix}_foot_rivet", (x, y, 0.031), 0.010, silver)

    # Four leg stubs (all to base_top_z, straight up — the seat/back now lives
    # on the swivel assembly above).
    _add_tube(frame, "front_leg_0", (left_x, front_y, 0.025), (left_x, front_y, base_top_z), tube_r, metal)
    _add_tube(frame, "front_leg_1", (right_x, front_y, 0.025), (right_x, front_y, base_top_z), tube_r, metal)
    _add_tube(frame, "rear_back_post_0", (left_x, rear_y, 0.025), (left_x, rear_y, base_top_z), tube_r, metal)
    _add_tube(frame, "rear_back_post_1", (right_x, rear_y, 0.025), (right_x, rear_y, base_top_z), tube_r, metal)

    # Base X-frame: two diagonal tubes in the horizontal plane that cross at the
    # hub centre, giving the folding base a visible top frame.
    _add_tube(frame, "base_xframe_0", (left_x, front_y, base_top_z), (right_x, rear_y, base_top_z), tube_r, metal)
    _add_tube(frame, "base_xframe_1", (right_x, front_y, base_top_z), (left_x, rear_y, base_top_z), tube_r, metal)

    # Lower cross members and fixed diagonal braces (same as parent).
    _add_tube(frame, "rear_lower_cross", (left_x, rear_y, 0.10), (right_x, rear_y, 0.10), tube_r, metal)
    _add_tube(frame, "front_lower_cross", (left_x, front_y, 0.10), (right_x, front_y, 0.10), tube_r, metal)
    # Fixed diagonal brace members stay within the base frame height
    # (below the swivel spoke endpoints at seat_z).
    brace_top_z = base_top_z - 0.08  # 0.37
    _add_tube(frame, "front_fixed_cross", (left_x, front_y + 0.018, brace_top_z), (right_x, front_y + 0.018, 0.070), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_0", (left_x + 0.018, front_y, brace_top_z), (left_x + 0.018, rear_y, 0.075), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_1", (right_x - 0.018, front_y, brace_top_z), (right_x - 0.018, rear_y, 0.075), tube_r, metal)

    # Pivot pins and clevis-like brackets for the articulated folding braces.
    front_pivot_y = front_y - 0.080
    _add_tube(frame, "front_pivot_pin", (-0.002, front_pivot_y - 0.032, 0.300), (-0.002, front_pivot_y + 0.032, 0.300), 0.018, black_plastic)
    _add_tube(frame, "front_pivot_support", (0.0, front_pivot_y + 0.026, 0.300), (0.0, front_y, base_top_z), 0.010, black_plastic)
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

    # Central swivel bearing hub (dark-green accent).  A short cylinder rising
    # from the base X-frame crossing point.  Radius is sized to contact the
    # collar torus inner surface (torus R=0.060, tube r=0.012 → inner R=0.048).
    frame.visual(
        Cylinder(radius=0.050, length=0.06),
        origin=Origin(xyz=hub_xyz),
        material=dark_green,
        name="swivel_hub",
    )

    # =========================================================================
    # SEAT SWIVEL ASSEMBLY
    # =========================================================================
    # The swivel part frame coincides with the articulation frame at hub_xyz.
    # All swivel visual coordinates are LOCAL to the hub centre.
    swivel = model.part("seat_swivel")
    _dy = 0.03   # local_y = world_y + 0.03  (hub y-offset)
    _dz = -0.46  # local_z = world_z - 0.46  (hub z-offset)

    def L(x, y, z):
        """Convert world coordinate to swivel-part local."""
        return (x, y + _dy, z + _dz)

    # Visible bearing collar — dark-green torus ring that wraps around the hub.
    collar_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.060, tube=0.012, radial_segments=20, tubular_segments=36),
        "swivel_collar",
    )
    swivel.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_green,
        name="swivel_collar",
    )

    # Hidden structural disk that connects the collar torus body to the radial
    # spokes.  Sits inside the hub envelope (overlap is allowed below).
    swivel.visual(
        Cylinder(radius=spoke_clearance, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_green,
        name="swivel_bearing_disk",
    )

    # Four radial spokes from the bearing area out to the seat-rail corners.
    seat_corners = (
        (L(left_x, front_y, seat_z), "swivel_spoke_fl"),
        (L(right_x, front_y, seat_z), "swivel_spoke_fr"),
        (L(left_x, rear_y, seat_z + 0.05), "swivel_spoke_rl"),
        (L(right_x, rear_y, seat_z + 0.05), "swivel_spoke_rr"),
    )
    hub_local = (0.0, 0.0, 0.0)
    for corner, name in seat_corners:
        start = _spoke_start(hub_local, corner, spoke_clearance)
        _add_tube(swivel, name, start, corner, tube_r, metal)

    # Seat perimeter rails (now part of the rotating seat frame).
    _add_tube(swivel, "front_seat_rail", L(left_x, front_y, seat_z), L(right_x, front_y, seat_z), tube_r, metal)
    _add_tube(swivel, "rear_seat_rail", L(left_x, rear_y, seat_z + 0.05), L(right_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(swivel, "side_seat_rail_0", L(left_x, front_y, seat_z), L(left_x, rear_y, seat_z + 0.05), tube_r, metal)
    _add_tube(swivel, "side_seat_rail_1", L(right_x, front_y, seat_z), L(right_x, rear_y, seat_z + 0.05), tube_r, metal)

    # Back uprights (upper portion of the rear posts, from seat rail to top).
    _add_tube(swivel, "back_upright_0", L(left_x, rear_y, seat_z), L(left_x, rear_y + 0.07, 1.12), tube_r, metal)
    _add_tube(swivel, "back_upright_1", L(right_x, rear_y, seat_z), L(right_x, rear_y + 0.07, 1.12), tube_r, metal)
    _add_tube(swivel, "top_back_rail", L(left_x, rear_y + 0.07, 1.12), L(right_x, rear_y + 0.07, 1.12), tube_r, metal)

    # Front arm posts (from seat rail up to arm height).
    _add_tube(swivel, "front_arm_post_0", L(left_x, front_y, seat_z), L(left_x, front_y + 0.025, arm_z_front), tube_r, metal)
    _add_tube(swivel, "front_arm_post_1", L(right_x, front_y, seat_z), L(right_x, front_y + 0.025, arm_z_front), tube_r, metal)

    # Arm sleeve cores (inside the sewn fabric arm-rest sleeves).
    _add_tube(swivel, "arm_sleeve_core_0", L(left_x, front_y + 0.025, 0.682), L(left_x, rear_y + 0.015, 0.704), 0.010, black_fabric)
    _add_tube(swivel, "arm_sleeve_core_1", L(right_x, front_y + 0.025, 0.682), L(right_x, rear_y + 0.015, 0.704), 0.010, black_fabric)

    # --- Sling seat ---
    _add_box(swivel, "seat_black_left", (0.17, 0.68, 0.020), L(-0.300, -0.035, seat_z + 0.012), black_fabric)
    _add_box(swivel, "seat_black_right", (0.17, 0.68, 0.020), L(0.300, -0.035, seat_z + 0.012), black_fabric)
    _add_box(swivel, "seat_gray_center", (0.42, 0.68, 0.018), L(0.0, -0.035, seat_z + 0.014), gray_fabric)
    _add_box(swivel, "seat_front_roll", (0.90, 0.040, 0.035), L(0.0, front_y - 0.015, seat_z + 0.022), black_fabric)
    _add_box(swivel, "seat_rear_lap", (0.84, 0.035, 0.030), L(0.0, rear_y + 0.010, seat_z + 0.050), black_fabric)
    _add_box(swivel, "seat_piping_0", (0.012, 0.650, 0.010), L(-0.205, -0.035, seat_z + 0.031), orange_trim)
    _add_box(swivel, "seat_piping_1", (0.012, 0.650, 0.010), L(0.205, -0.035, seat_z + 0.031), orange_trim)
    _add_box(swivel, "front_edge_stitch", (0.82, 0.008, 0.010), L(0.0, front_y - 0.038, seat_z + 0.044), white)

    # --- Tall padded back ---
    back_roll = -0.15
    _add_box(swivel, "back_gray_center", (0.42, 0.028, 0.690), L(0.0, 0.285, 0.825), gray_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_black_side_0", (0.18, 0.030, 0.700), L(-0.305, 0.285, 0.825), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_black_side_1", (0.18, 0.030, 0.700), L(0.305, 0.285, 0.825), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_top_pillow", (0.82, 0.045, 0.110), L(0.0, 0.340, 1.150), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_lower_band", (0.78, 0.033, 0.090), L(0.0, 0.232, 0.565), black_fabric, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_piping_0", (0.012, 0.026, 0.650), L(-0.210, 0.246, 0.850), orange_trim, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_piping_1", (0.012, 0.026, 0.650), L(0.210, 0.246, 0.850), orange_trim, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_top_stitch", (0.78, 0.030, 0.018), L(0.0, 0.336, 1.155), white, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_logo_mark", (0.090, 0.045, 0.035), L(0.0, 0.285, 1.040), white, rpy=(back_roll, 0.0, 0.0))
    _add_box(swivel, "back_logo_text", (0.160, 0.045, 0.022), L(0.0, 0.285, 0.995), white, rpy=(back_roll, 0.0, 0.0))

    # --- Arm rests, cup holder, side pocket ---
    for x, side, outer_sign in ((left_x, "0", -1.0), (right_x, "1", 1.0)):
        _add_box(swivel, f"arm_rest_top_{side}", (0.165, 0.585, 0.022), L(x, -0.040, 0.703), black_fabric)
        _add_box(swivel, f"arm_rest_outer_skirt_{side}", (0.036, 0.560, 0.110), L(x + outer_sign * 0.072, -0.030, 0.660), black_fabric)
        _add_box(swivel, f"arm_rest_front_cuff_{side}", (0.170, 0.050, 0.070), L(x, front_y + 0.018, 0.676), black_fabric)
        _add_box(swivel, f"arm_rest_rear_cuff_{side}", (0.170, 0.055, 0.075), L(x, rear_y + 0.025, 0.724), black_fabric)
        _add_box(swivel, f"arm_rest_outer_stitch_{side}", (0.010, 0.545, 0.010), L(x + outer_sign * 0.086, -0.035, 0.716), white)
        _add_box(swivel, f"arm_rest_inner_stitch_{side}", (0.010, 0.520, 0.010), L(x - outer_sign * 0.054, -0.030, 0.715), white)
        _add_box(swivel, f"arm_rest_front_bar_tack_{side}", (0.145, 0.010, 0.012), L(x, front_y + 0.046, 0.713), orange_trim)
        _add_box(swivel, f"arm_rest_rear_bar_tack_{side}", (0.145, 0.010, 0.012), L(x, rear_y - 0.004, 0.742), orange_trim)

    cup_ring = mesh_from_geometry(TorusGeometry(radius=0.055, tube=0.006, radial_segments=20, tubular_segments=36), "cup_holder_ring")
    swivel.visual(cup_ring, origin=Origin(xyz=L(left_x - 0.005, -0.245, 0.724)), material=black_plastic, name="cup_holder_ring")
    swivel.visual(Cylinder(radius=0.050, length=0.100), origin=Origin(xyz=L(left_x, -0.245, 0.655)), material=mesh_mat, name="cup_holder_sleeve")
    _add_box(swivel, "side_pocket_front", (0.160, 0.018, 0.230), L(left_x - 0.012, -0.242, 0.535), mesh_mat)
    _add_box(swivel, "side_pocket_outer", (0.020, 0.180, 0.230), L(left_x - 0.082, -0.160, 0.535), mesh_mat)
    _add_box(swivel, "side_pocket_label", (0.045, 0.014, 0.072), L(left_x - 0.012, -0.252, 0.565), white)

    # =========================================================================
    # ARTICULATED CROSS BRACES (unchanged from parent — still fold on the base)
    # =========================================================================
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

    # =========================================================================
    # ARTICULATIONS
    # =========================================================================
    # Folding brace joints (unchanged from parent — base still folds).
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

    # New 360-degree swivel joint between base and seat assembly.
    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=swivel,
        origin=Origin(xyz=hub_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    swivel = object_model.get_part("seat_swivel")
    front = object_model.get_part("front_cross_brace")
    side0 = object_model.get_part("side_cross_brace_0")
    side1 = object_model.get_part("side_cross_brace_1")

    swivel_joint = object_model.get_articulation("base_to_swivel")
    front_joint = object_model.get_articulation("frame_to_front_cross")
    side_joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    side_joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # --- Pivot-hardware overlap allowances (unchanged from parent) ---
    ctx.allow_overlap(
        frame, front, elem_a="front_pivot_pin", elem_b="front_pivot_collar",
        reason="The moving scissor brace collar is intentionally captured around the fixed front pivot pin.",
    )
    ctx.allow_overlap(
        frame, side0, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, side1, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1",
        reason="The side scissor collar is intentionally captured by the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame, side0, elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, side1, elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1",
        reason="The rivet head intentionally clamps the outside of the side scissor pivot collar.",
    )
    ctx.allow_overlap(
        frame, front, elem_a="front_pivot_rivet", elem_b="front_pivot_collar",
        reason="The front rivet head intentionally captures the rotating scissor collar on the pivot.",
    )
    ctx.allow_overlap(
        frame, side0, elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )
    ctx.allow_overlap(
        frame, side1, elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1",
        reason="The side rivet head intentionally passes through the moving brace tube at the scissor joint.",
    )

    # --- Swivel bearing overlap allowances ---
    ctx.allow_overlap(
        frame, swivel, elem_a="swivel_hub", elem_b="swivel_collar",
        reason="The bearing collar wraps around the hub as a visible turntable ring.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="swivel_hub", elem_b="swivel_bearing_disk",
        reason="The bearing disk is captured inside the hub envelope to structurally connect the collar and radial spokes.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_0", elem_b="swivel_collar",
        reason="The base X-frame diagonal passes through the bearing hub crossing directly under the collar torus.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_1", elem_b="swivel_collar",
        reason="The base X-frame diagonal passes through the bearing hub crossing directly under the collar torus.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_0", elem_b="swivel_bearing_disk",
        reason="The base X-frame diagonal passes under the bearing disk at the hub crossing.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_1", elem_b="swivel_bearing_disk",
        reason="The base X-frame diagonal passes under the bearing disk at the hub crossing.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_0", elem_b="swivel_spoke_fl",
        reason="The X-frame diagonal and spoke cross near the bearing hub at the same elevation.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_0", elem_b="swivel_spoke_rr",
        reason="The X-frame diagonal and spoke cross near the bearing hub at the same elevation.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_1", elem_b="swivel_spoke_fr",
        reason="The X-frame diagonal and spoke cross near the bearing hub at the same elevation.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="base_xframe_1", elem_b="swivel_spoke_rl",
        reason="The X-frame diagonal and spoke cross near the bearing hub at the same elevation.",
    )
    ctx.allow_overlap(
        frame, swivel, elem_a="swivel_hub", elem_b="seat_gray_center",
        reason="The swivel bearing hub rises through the seat platform base as in a real rotating camp-chair mechanism.",
    )

    # --- Metadata ---
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # --- Swivel joint identity (prompt-specific) ---
    ctx.check(
        "base_to_swivel is CONTINUOUS about vertical Z axis",
        swivel_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(swivel_joint.axis[2]) > 0.99
        and abs(swivel_joint.axis[0]) < 0.01
        and abs(swivel_joint.axis[1]) < 0.01,
        details=f"type={swivel_joint.articulation_type} axis={swivel_joint.axis}",
    )

    # --- Bearing collar retention ---
    ctx.expect_overlap(
        frame, swivel, axes="xy", min_overlap=0.02,
        elem_a="swivel_hub", elem_b="swivel_collar",
        name="bearing collar retained on hub in XY",
    )

    # --- Swivel rotation proof (track off-axis arm rest via AABB) ---
    rest_aabb = ctx.part_world_aabb(swivel)
    with ctx.pose({swivel_joint: math.pi / 2}):
        rotated_aabb = ctx.part_world_aabb(swivel)
        ctx.expect_overlap(
            frame, swivel, axes="xy", min_overlap=0.02,
            elem_a="swivel_hub", elem_b="swivel_collar",
            name="bearing collar retained on hub after 90° swivel",
        )
    ctx.check(
        "swivel rotation changes seat assembly footprint",
        rest_aabb is not None
        and rotated_aabb is not None
        and (
            abs((rotated_aabb[1][0] - rotated_aabb[0][0]) - (rest_aabb[1][0] - rest_aabb[0][0])) > 0.02
            or abs((rotated_aabb[1][1] - rotated_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.02
        ),
        details=f"rest_aabb={rest_aabb} rotated_aabb={rotated_aabb}",
    )

    # --- Brace retention (same as parent) ---
    ctx.expect_overlap(frame, front, axes="yz", min_overlap=0.020, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_0", elem_b="side_pivot_collar_0", name="left side brace collar retained on pivot")
    ctx.expect_overlap(frame, side1, axes="xz", min_overlap=0.020, elem_a="side_pivot_pin_1", elem_b="side_pivot_collar_1", name="right side brace collar retained on pivot")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_pivot_collar_0", name="left rivet head clamps pivot collar")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_pivot_collar_1", name="right rivet head clamps pivot collar")
    ctx.expect_overlap(frame, front, axes="xz", min_overlap=0.010, elem_a="front_pivot_rivet", elem_b="front_pivot_collar", name="front rivet head captures pivot collar")
    ctx.expect_overlap(frame, side0, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_0", elem_b="side_moving_tube_0", name="left rivet passes through moving brace")
    ctx.expect_overlap(frame, side1, axes="yz", min_overlap=0.010, elem_a="side_pivot_rivet_1", elem_b="side_moving_tube_1", name="right rivet passes through moving brace")
    ctx.expect_gap(frame, front, axis="z", max_penetration=0.05, elem_a="front_pivot_pin", elem_b="front_pivot_collar", name="front pivot overlap remains localized")

    # --- Folding mechanism (same as parent) ---
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
