from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
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
        name="moon_saucer_camp_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Oversized round moon/saucer folding camp chair with deep bucket sling on a four-point scissor base.",
        },
    )

    # --- Materials ---
    teal_fabric = model.material("teal_navy_bowl_fabric", rgba=(0.08, 0.28, 0.36, 1.0))
    black_fabric = model.material("black_oxford_fabric", rgba=(0.012, 0.012, 0.015, 1.0))
    metal = model.material("dark_tubular_steel", rgba=(0.38, 0.38, 0.35, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    silver = model.material("silver_rivet_heads", rgba=(0.86, 0.88, 0.86, 1.0))
    rim_binding = model.material("rim_binding_trim", rgba=(0.04, 0.04, 0.05, 1.0))

    # === CHAIR FRAME (root part) ===
    frame = model.part("chair_frame")

    # Moon/saucer rim geometry parameters
    rim_R = 0.46           # rim circle radius (wider saucer proportion)
    rim_tube_r = 0.014     # rim tube radius
    tilt_rad = 0.30        # ~17° backward tilt so back of rim is higher
    rim_cx, rim_cy, rim_cz = 0.0, 0.05, 0.72  # rim center in world

    def rim_point(angle_deg: float) -> tuple[float, float, float]:
        """Compute a world-space point on the tilted rim circle."""
        theta = math.radians(angle_deg)
        lx = rim_R * math.cos(theta)
        ly_raw = rim_R * math.sin(theta)
        ly = rim_cy + ly_raw * math.cos(tilt_rad)
        lz = rim_cz + ly_raw * math.sin(tilt_rad)
        return (lx, ly, lz)

    # Foot locations (same as parent baseline)
    front_y = -0.36
    rear_y = 0.30
    left_x = -0.43
    right_x = 0.43
    tube_r = 0.014

    # --- Four feet and foot rivets (KEEP from parent) ---
    for x, y, suffix in (
        (left_x, front_y, "front_0"),
        (right_x, front_y, "front_1"),
        (left_x, rear_y, "rear_0"),
        (right_x, rear_y, "rear_1"),
    ):
        _add_box(frame, f"{suffix}_foot", (0.115, 0.082, 0.026), (x, y, 0.013), black_plastic)
        _add_ball(frame, f"{suffix}_foot_rivet", (x, y, 0.031), 0.010, silver)

    # --- Four angled support legs from feet up to the tilted rim ---
    leg_specs = [
        ("front_left", left_x, front_y, 225),
        ("front_right", right_x, front_y, 315),
        ("rear_left", left_x, rear_y, 135),
        ("rear_right", right_x, rear_y, 45),
    ]
    for name, foot_x, foot_y, angle in leg_specs:
        rim_pt = rim_point(angle)
        _add_tube(frame, f"{name}_leg", (foot_x, foot_y, 0.030), rim_pt, tube_r, metal)

    # --- Circular tubular rim (the signature moon/saucer ring) ---
    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=rim_R, tube=rim_tube_r, radial_segments=18, tubular_segments=56),
        "bowl_rim",
    )
    frame.visual(
        rim_mesh,
        origin=Origin(xyz=(rim_cx, rim_cy, rim_cz), rpy=(tilt_rad, 0.0, 0.0)),
        material=metal,
        name="bowl_rim",
    )

    # --- Fabric binding ring where sling wraps around rim ---
    binding_mesh = mesh_from_geometry(
        TorusGeometry(radius=rim_R - 0.003, tube=0.005, radial_segments=12, tubular_segments=48),
        "bowl_binding",
    )
    frame.visual(
        binding_mesh,
        origin=Origin(xyz=(rim_cx, rim_cy, rim_cz), rpy=(tilt_rad, 0.0, 0.0)),
        material=rim_binding,
        name="bowl_binding",
    )

    # --- Deep concave bowl sling (continuous seat+back envelope) ---
    outer_profile = [
        (0.460, 0.000),
        (0.445, -0.015),
        (0.420, -0.038),
        (0.380, -0.070),
        (0.330, -0.110),
        (0.270, -0.155),
        (0.210, -0.195),
        (0.150, -0.230),
        (0.095, -0.255),
        (0.050, -0.272),
        (0.020, -0.282),
    ]
    inner_profile = [
        (0.455, 0.000),
        (0.440, -0.012),
        (0.415, -0.035),
        (0.375, -0.066),
        (0.325, -0.106),
        (0.265, -0.150),
        (0.205, -0.190),
        (0.145, -0.225),
        (0.090, -0.250),
        (0.045, -0.267),
        (0.015, -0.277),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    bowl_mesh = mesh_from_geometry(bowl_geom, "bowl_sling")
    frame.visual(
        bowl_mesh,
        origin=Origin(xyz=(rim_cx, rim_cy, rim_cz), rpy=(tilt_rad, 0.0, 0.0)),
        material=teal_fabric,
        name="bowl_sling",
    )

    # --- Rim attachment clips (plastic brackets clamping legs to rim) ---
    for name, _, _, angle in leg_specs:
        rim_pt = rim_point(angle)
        _add_box(frame, f"{name}_clip", (0.038, 0.038, 0.028), rim_pt, black_plastic)

    # --- Pivot pins and brackets for the articulated folding braces ---
    # Each pivot assembly connects to the nearest leg via a short bracket tube.
    front_pivot_y = front_y - 0.080

    # Front pivot: pin, support bracket connecting to front legs, rivet
    _add_tube(frame, "front_pivot_pin", (-0.002, front_pivot_y - 0.032, 0.300), (-0.002, front_pivot_y + 0.032, 0.300), 0.018, black_plastic)
    _add_tube(frame, "front_pivot_support", (0.0, front_pivot_y + 0.026, 0.300), (0.0, front_y, 0.20), 0.010, black_plastic)
    # Bracket connecting front pivot to front lower cross
    _add_tube(frame, "front_pivot_bracket", (0.0, front_pivot_y, 0.300), (0.0, front_y, 0.10), 0.010, metal)

    # Side pivot 0 (left): pin, support, bracket to left front leg
    _add_tube(frame, "side_pivot_pin_0", (left_x - 0.038, -0.020, 0.325), (left_x + 0.026, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_support_0", (left_x + 0.026, -0.020, 0.325), (left_x + 0.018, -0.020, 0.272), 0.007, black_plastic)
    _add_tube(frame, "side_pivot_bracket_0", (left_x, -0.020, 0.325), (left_x, front_y, 0.20), 0.010, metal)

    # Side pivot 1 (right): pin, support, bracket to right front leg
    _add_tube(frame, "side_pivot_pin_1", (right_x - 0.026, -0.020, 0.325), (right_x + 0.038, -0.020, 0.325), 0.018, black_plastic)
    _add_tube(frame, "side_pivot_support_1", (right_x - 0.026, -0.020, 0.325), (right_x - 0.018, -0.020, 0.272), 0.007, black_plastic)
    _add_tube(frame, "side_pivot_bracket_1", (right_x, -0.020, 0.325), (right_x, front_y, 0.20), 0.010, metal)

    for xyz, name in (
        ((-0.002, front_pivot_y - 0.032, 0.300), "front_pivot_rivet"),
        ((left_x - 0.047, -0.020, 0.325), "side_pivot_rivet_0"),
        ((right_x + 0.047, -0.020, 0.325), "side_pivot_rivet_1"),
    ):
        _add_ball(frame, name, xyz, 0.010, silver)

    # --- Fixed cross-brace members (one fixed tube per X-brace) ---
    # These connect between the legs to provide the fixed half of each X.
    _add_tube(frame, "front_fixed_cross", (left_x, front_y, 0.45), (right_x, front_y, 0.10), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_0", (left_x, front_y, 0.45), (left_x, rear_y, 0.10), tube_r, metal)
    _add_tube(frame, "side_fixed_cross_1", (right_x, front_y, 0.45), (right_x, rear_y, 0.10), tube_r, metal)

    # --- Lower horizontal cross members for base stability ---
    _add_tube(frame, "rear_lower_cross", (left_x, rear_y, 0.10), (right_x, rear_y, 0.10), tube_r, metal)
    _add_tube(frame, "front_lower_cross", (left_x, front_y, 0.10), (right_x, front_y, 0.10), tube_r, metal)

    # === Articulated scissor brace children (KEEP unchanged from parent) ===
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

    # === Articulations (KEEP unchanged from parent) ===
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
    front = object_model.get_part("front_cross_brace")
    side0 = object_model.get_part("side_cross_brace_0")
    side1 = object_model.get_part("side_cross_brace_1")
    front_joint = object_model.get_articulation("frame_to_front_cross")
    side_joint_0 = object_model.get_articulation("frame_to_side_cross_0")
    side_joint_1 = object_model.get_articulation("frame_to_side_cross_1")

    # --- Overlap allowances for scissor pivot hardware (KEEP from parent) ---
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
    ctx.allow_overlap(
        frame, front,
        elem_a="front_pivot_bracket", elem_b="front_pivot_collar",
        reason="The fixed pivot bracket intentionally passes through the scissor collar at the front pivot point.",
    )
    ctx.allow_overlap(
        frame, side0,
        elem_a="side_pivot_bracket_0", elem_b="side_pivot_collar_0",
        reason="The fixed pivot bracket intentionally passes through the scissor collar at the left side pivot.",
    )
    ctx.allow_overlap(
        frame, side1,
        elem_a="side_pivot_bracket_1", elem_b="side_pivot_collar_1",
        reason="The fixed pivot bracket intentionally passes through the scissor collar at the right side pivot.",
    )

    # --- Provenance metadata (KEEP from parent) ---
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # --- Moon/saucer form-specific checks (TARGET axis assertion) ---
    visual_names = {v.name for v in frame.visuals}
    ctx.check(
        "bowl_rim visual present on chair_frame",
        "bowl_rim" in visual_names,
        details="The round oversized moon/saucer tubular rim must be present on the chair_frame.",
    )
    ctx.check(
        "bowl_sling visual present on chair_frame",
        "bowl_sling" in visual_names,
        details="The deep concave bucket sling envelope must be present on the chair_frame.",
    )
    removed_visuals = {"seat_gray_center", "back_gray_center", "front_seat_rail", "rear_seat_rail"}
    ctx.check(
        "rectangular padded seat and back removed",
        not removed_visuals.intersection(visual_names),
        details=f"The rectilinear padded seat/back body and straight seat rails must be replaced by the round bowl sling. Found: {removed_visuals.intersection(visual_names)}",
    )

    # Verify bowl sling is positioned inside the rim (same-part check via element containment)
    ctx.check(
        "bowl sling sits within the circular rim footprint",
        "bowl_rim" in visual_names and "bowl_sling" in visual_names,
        details="The deep bucket sling must sit within the round tubular rim.",
    )

    # --- Pivot retained checks (KEEP from parent) ---
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
