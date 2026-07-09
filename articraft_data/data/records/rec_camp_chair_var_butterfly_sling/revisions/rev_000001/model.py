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
    tube_from_spline_points,
)


# ---------------------------------------------------------------------------
# Geometry helpers (preserved from parent)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Butterfly/sling loop path builder
# ---------------------------------------------------------------------------

def _loop_path(
    foot_x: float,
    foot_y: float,
    foot_z: float,
    tip_x: float,
    tip_y: float,
    tip_z: float,
    bar_z_offset: float,
    n_mid: int = 2,
) -> list[tuple[float, float, float]]:
    """Build a symmetric U-loop path from left foot -> left tip -> crossbar -> right tip -> right foot.

    Uses sparse intermediate points to minimize Catmull-Rom spline bulging.
    The loop has two legs converging from the feet toward a top crossbar.
    """
    pts: list[tuple[float, float, float]] = []

    # Left foot
    pts.append((foot_x, foot_y, foot_z))

    # Left leg: one midpoint between foot and tip
    mid_y = (foot_y + tip_y) * 0.5
    mid_z = (foot_z + tip_z) * 0.5
    pts.append((foot_x + (tip_x - foot_x) * 0.5, mid_y, mid_z))

    # Left tip
    pts.append((tip_x, tip_y, tip_z))

    # Crossbar: left tip -> centre -> right tip (3 points with a slight arc)
    bar_y = tip_y + 0.005
    pts.append((tip_x * 0.35, bar_y, tip_z + bar_z_offset * 0.7))
    pts.append((0.0, bar_y, tip_z + bar_z_offset))
    pts.append((-tip_x * 0.35, bar_y, tip_z + bar_z_offset * 0.7))

    # Right tip
    pts.append((-tip_x, tip_y, tip_z))

    # Right leg: one midpoint
    pts.append((-foot_x - (tip_x - foot_x) * 0.5, mid_y, mid_z))

    # Right foot
    pts.append((-foot_x, foot_y, foot_z))

    return pts


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="folding_camping_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Butterfly/sling folding camp chair: two crossed tubular loop frames carrying a continuous corner-pocket sling.",
            "variant": "butterfly_sling_rust_orange",
        },
    )

    # -- materials ----------------------------------------------------------
    rust_canvas = model.material("rust_orange_canvas", rgba=(0.82, 0.38, 0.10, 1.0))
    dark_steel = model.material("powder_coated_steel_tube", rgba=(0.22, 0.22, 0.21, 1.0))
    black_rubber = model.material("black_rubber_feet", rgba=(0.02, 0.02, 0.02, 1.0))
    zinc_bolt = model.material("zinc_plated_hardware", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_binding = model.material("dark_edge_binding", rgba=(0.08, 0.05, 0.03, 1.0))

    tube_r = 0.011  # 22 mm OD tube

    # =====================================================================
    # Key dimensions for the two crossed loops.
    #
    # Back loop (wider): foot at front-bottom, tip at back-top.
    # Front loop (narrower, nests inside): foot at back-bottom, tip at front-mid.
    # They cross at the pivot bolt height.
    # =====================================================================

    # Back loop parameters (world coords)
    back_foot_x = 0.25
    back_foot_y = -0.28
    back_foot_z = 0.020
    back_tip_x = 0.21
    back_tip_y = 0.15
    back_tip_z = 0.92

    # Front loop parameters (will be expressed in local frame of front_cross_brace)
    front_foot_x = 0.17
    front_foot_y = 0.28   # in world
    front_foot_z = 0.020
    front_tip_x = 0.15
    front_tip_y = -0.10   # in world
    front_tip_z = 0.70

    # Pivot point (world) — where the two loops cross
    pivot_xyz = (0.0, (back_foot_y + front_foot_y) * 0.5 * 0.15, (back_foot_z + front_tip_z) * 0.5 + 0.18)
    pivot_xyz = (0.0, -0.025, 0.554)  # computed from linear interpolation of both legs

    # Part-local offsets for front loop (front_cross_brace origin = pivot)
    py_off = -pivot_xyz[1]   # +0.025
    pz_off = -pivot_xyz[2]   # -0.554

    # =====================================================================
    # chair_frame (root) — back loop
    # =====================================================================
    frame = model.part("chair_frame")

    # Back-loop spline path
    back_loop_pts = _loop_path(
        foot_x=-back_foot_x, foot_y=back_foot_y, foot_z=back_foot_z,
        tip_x=-back_tip_x, tip_y=back_tip_y, tip_z=back_tip_z,
        bar_z_offset=0.020,
    )
    back_tube_geom = tube_from_spline_points(
        back_loop_pts,
        radius=tube_r,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    back_tube_mesh = mesh_from_geometry(back_tube_geom, "back_loop_frame")
    frame.visual(back_tube_mesh, origin=Origin(), material=dark_steel, name="back_loop_frame")

    # Lower stability crossbar between the two back-loop legs near the feet.
    _add_tube(frame, "back_lower_cross",
              (-back_foot_x + 0.006, back_foot_y + 0.05, back_foot_z + 0.09),
              (back_foot_x - 0.006, back_foot_y + 0.05, back_foot_z + 0.09),
              tube_r * 0.85, dark_steel)

    # Rubber foot caps on the two back-loop feet.
    for i, sign in enumerate([-1.0, 1.0]):
        _add_ball(frame, f"rear_foot_{i}",
                  (sign * back_foot_x, back_foot_y, back_foot_z),
                  0.017, black_rubber)

    # Pivot bolt at the crossing point (shared with the front loop).
    bolt_half = back_foot_x + 0.015  # spans across both back-loop legs and beyond
    _add_tube(frame, "pivot_bolt",
              (-bolt_half, pivot_xyz[1], pivot_xyz[2]),
              (bolt_half, pivot_xyz[1], pivot_xyz[2]),
              0.007, zinc_bolt)
    _add_ball(frame, "pivot_bolt_head_0",
              (-bolt_half - 0.005, pivot_xyz[1], pivot_xyz[2]),
              0.010, zinc_bolt)
    _add_ball(frame, "pivot_bolt_head_1",
              (bolt_half + 0.005, pivot_xyz[1], pivot_xyz[2]),
              0.010, zinc_bolt)

    # -- Sling fabric (rust-orange canvas) ---------------------------------
    # The continuous sling hooks onto the four upper tube tips.
    # Backrest panel: tilted, connecting the two back tips.
    # Seat panel: roughly horizontal, connecting back tips to front tips.

    # Backrest: from back crossbar area down toward the seat crease
    backrest_y = back_tip_y - 0.04
    backrest_z = (back_tip_z + front_tip_z) * 0.5 + 0.12
    backrest_h = back_tip_z - front_tip_z + 0.08
    _add_box(
        frame, "seat_gray_center",
        (back_tip_x * 2 + 0.02, 0.016, backrest_h),
        (0.0, backrest_y, backrest_z),
        rust_canvas,
        rpy=(-0.20, 0.0, 0.0),
    )

    # Seat panel: roughly horizontal between the two loop tips
    seat_y = (back_tip_y + front_tip_y) * 0.5
    seat_z = front_tip_z - 0.02
    seat_depth = back_tip_y - front_tip_y + 0.06
    _add_box(
        frame, "seat_panel",
        (back_tip_x * 2 + 0.02, seat_depth, 0.016),
        (0.0, seat_y, seat_z),
        rust_canvas,
        rpy=(0.08, 0.0, 0.0),
    )

    # Edge binding strips that overlap with both the sling and the frame tubes.
    # These serve as the physical connection between sling fabric and frame.
    # Left binding: spans from the seat panel edge up to the back tip, touching the back tube.
    sling_half_w = back_tip_x + 0.012  # extends past the back tip tube to ensure contact
    _add_box(frame, "sling_binding_left",
             (0.016, seat_depth * 0.9, backrest_h * 0.85),
             (-sling_half_w, (backrest_y + seat_y) * 0.5, (backrest_z + seat_z) * 0.5),
             dark_binding,
             rpy=(-0.10, 0.0, 0.0))
    _add_box(frame, "sling_binding_right",
             (0.016, seat_depth * 0.9, backrest_h * 0.85),
             (sling_half_w, (backrest_y + seat_y) * 0.5, (backrest_z + seat_z) * 0.5),
             dark_binding,
             rpy=(-0.10, 0.0, 0.0))

    # Top binding connecting across the back crossbar (touches both back tips).
    _add_box(frame, "sling_binding_top",
             (back_tip_x * 2 + 0.04, 0.025, 0.025),
             (0.0, back_tip_y + 0.005, back_tip_z + 0.015),
             dark_binding)

    # Front binding connecting across the front tip area.
    # Extended in Y to bridge the gap between front_tip_y and seat_panel.
    binding_y_extent = 0.12  # extends from front_tip_y toward seat_panel
    _add_box(frame, "sling_binding_front",
             (front_tip_x * 2 + 0.04, binding_y_extent, 0.030),
             (0.0, front_tip_y + binding_y_extent * 0.5, front_tip_z - 0.002),
             dark_binding)

    # Corner-pocket wraps where the sling hooks over each back tube tip.
    for i, sign in enumerate([-1.0, 1.0]):
        px = sign * back_tip_x
        _add_box(frame, f"back_pocket_{i}",
                 (0.055, 0.045, 0.065),
                 (px, back_tip_y, back_tip_z),
                 rust_canvas)

    # =====================================================================
    # front_cross_brace (child) — front loop
    # Part origin at the pivot so the revolute joint is at (0,0,0) locally.
    # =====================================================================
    front_brace = model.part("front_cross_brace")

    # Front-loop spline path in part-local frame
    front_loop_pts = _loop_path(
        foot_x=-front_foot_x, foot_y=front_foot_y + py_off, foot_z=front_foot_z + pz_off,
        tip_x=-front_tip_x, tip_y=front_tip_y + py_off, tip_z=front_tip_z + pz_off,
        bar_z_offset=0.015,
    )
    front_tube_geom = tube_from_spline_points(
        front_loop_pts,
        radius=tube_r,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    front_tube_mesh = mesh_from_geometry(front_tube_geom, "front_loop_frame")
    front_brace.visual(front_tube_mesh, origin=Origin(), material=dark_steel, name="front_loop_frame")

    # Lower stability crossbar on the front loop (local frame).
    # Position computed from the front loop leg at z_world ≈ 0.11.
    _add_tube(front_brace, "front_lower_cross",
              (-front_foot_x + 0.003, front_foot_y - 0.05 + py_off, front_foot_z + 0.09 + pz_off),
              (front_foot_x - 0.003, front_foot_y - 0.05 + py_off, front_foot_z + 0.09 + pz_off),
              tube_r * 0.85, dark_steel)

    # Rubber foot caps on the two front-loop feet (local frame).
    for i, sign in enumerate([-1.0, 1.0]):
        _add_ball(front_brace, f"front_foot_{i}",
                  (sign * front_foot_x, front_foot_y + py_off, front_foot_z + pz_off),
                  0.017, black_rubber)

    # Pivot collar: sleeve spanning across both front loop legs at the crossing.
    # The front loop legs at the crossing are at x ≈ ±front_tip_x (≈ ±0.15).
    collar_half = front_foot_x + 0.01  # extends past both legs
    _add_tube(front_brace, "front_pivot_collar",
              (-collar_half, 0.0, 0.0), (collar_half, 0.0, 0.0),
              0.018, black_rubber)

    # Corner-pocket wraps on the front tips (local frame).
    for i, sign in enumerate([-1.0, 1.0]):
        px = sign * front_tip_x
        py = front_tip_y + py_off
        pz = front_tip_z + pz_off
        _add_box(front_brace, f"front_pocket_{i}",
                 (0.055, 0.045, 0.065),
                 (px, py, pz),
                 rust_canvas)

    # =====================================================================
    # Single folding pivot: where the two loops cross.
    # =====================================================================
    model.articulation(
        "frame_to_front_cross",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_brace,
        origin=Origin(xyz=pivot_xyz),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    front = object_model.get_part("front_cross_brace")
    pivot_joint = object_model.get_articulation("frame_to_front_cross")

    # -- provenance ---------------------------------------------------------
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # -- intentional overlap: pivot collar captured around bolt -------------
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_bolt",
        elem_b="front_pivot_collar",
        reason="The front loop pivot collar is intentionally captured around the fixed pivot bolt at the butterfly X crossing.",
    )
    ctx.expect_overlap(
        frame, front,
        axes="yz",
        min_overlap=0.012,
        elem_a="pivot_bolt",
        elem_b="front_pivot_collar",
        name="pivot collar retained on bolt at crossing",
    )

    # -- intentional overlap: sling seat draped over front tube tips -------
    ctx.allow_overlap(
        frame,
        front,
        elem_a="seat_panel",
        elem_b="front_loop_frame",
        reason="The sling seat fabric intentionally wraps over the front loop tube tips, forming the corner-pocket seat attachment.",
    )
    ctx.expect_contact(
        frame,
        front,
        elem_a="seat_panel",
        elem_b="front_loop_frame",
        name="sling seat rests on front loop tube tips",
    )

    # -- intentional overlap: sling seat fabric over front corner pockets --
    for i in range(2):
        ctx.allow_overlap(
            frame,
            front,
            elem_a="seat_panel",
            elem_b=f"front_pocket_{i}",
            reason=f"The sling seat panel sits on the front corner pocket wrap at tube tip {i}.",
        )
    ctx.expect_overlap(
        frame, front,
        axes="x",
        min_overlap=0.020,
        elem_a="seat_panel",
        elem_b="front_pocket_0",
        name="seat panel covers front pocket wrap at tip 0",
    )

    # -- intentional overlap: front binding wraps around front tube tips ---
    ctx.allow_overlap(
        frame,
        front,
        elem_a="sling_binding_front",
        elem_b="front_loop_frame",
        reason="The front sling binding wraps around the front loop tube tips to secure the sling attachment.",
    )
    ctx.expect_contact(
        frame,
        front,
        elem_a="sling_binding_front",
        elem_b="front_loop_frame",
        name="front binding contacts front loop at sling attachment",
    )

    # -- intentional overlap: binding and pocket both wrap tube tips -------
    for i in range(2):
        ctx.allow_overlap(
            frame,
            front,
            elem_a="sling_binding_front",
            elem_b=f"front_pocket_{i}",
            reason=f"The front binding and pocket wrap both enclose the front tube tip {i} in the sling attachment zone.",
        )
    ctx.expect_overlap(
        frame, front,
        axes="x",
        min_overlap=0.015,
        elem_a="sling_binding_front",
        elem_b="front_pocket_0",
        name="front binding and pocket overlap at tip 0 sling zone",
    )

    # -- intentional overlap: pivot bolt passes through front loop frame ---
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_bolt",
        elem_b="front_loop_frame",
        reason="The pivot bolt passes through the front loop frame at the rotation axis.",
    )
    ctx.expect_overlap(
        frame, front,
        axes="x",
        min_overlap=0.010,
        elem_a="pivot_bolt",
        elem_b="front_loop_frame",
        name="pivot bolt retained through front loop frame",
    )

    # -- butterfly X-loop structure ----------------------------------------
    ctx.check(
        "back_loop_frame visual exists on chair_frame",
        frame.get_visual("back_loop_frame") is not None,
        details="chair_frame must carry the continuous back loop tube",
    )
    ctx.check(
        "front_loop_frame visual exists on front_cross_brace",
        front.get_visual("front_loop_frame") is not None,
        details="front_cross_brace must carry the continuous front loop tube",
    )

    # -- sling fabric present ----------------------------------------------
    ctx.check(
        "sling fabric seat_gray_center present",
        frame.get_visual("seat_gray_center") is not None,
        details="continuous sling fabric must be present on chair_frame",
    )

    # -- folding articulation ----------------------------------------------
    ctx.check(
        "frame_to_front_cross is revolute with positive range",
        pivot_joint.articulation_type == ArticulationType.REVOLUTE
        and pivot_joint.motion_limits.lower == 0.0
        and pivot_joint.motion_limits.upper > 0.5,
        details=f"type={pivot_joint.articulation_type}, limits={pivot_joint.motion_limits}",
    )

    # Folding pose: confirm the front brace actually moves when folding.
    rest_aabb = ctx.part_world_aabb(front)
    with ctx.pose({pivot_joint: 0.60}):
        folded_aabb = ctx.part_world_aabb(front)
        ctx.expect_overlap(
            frame, front,
            axes="yz",
            min_overlap=0.010,
            elem_a="pivot_bolt",
            elem_b="front_pivot_collar",
            name="pivot still captured while folding",
        )

    ctx.check(
        "front_cross_brace changes shape when folding",
        rest_aabb is not None
        and folded_aabb is not None
        and abs((folded_aabb[1][2] - folded_aabb[0][2]) - (rest_aabb[1][2] - rest_aabb[0][2])) > 0.020,
        details=f"rest_z_span={rest_aabb}, folded_z_span={folded_aabb}",
    )

    # -- no leftover side braces from the parent ---------------------------
    all_part_names = {p.name for p in object_model.parts}
    ctx.check(
        "side cross braces removed (butterfly topology)",
        "side_cross_brace_0" not in all_part_names
        and "side_cross_brace_1" not in all_part_names,
        details=f"parts={sorted(all_part_names)}",
    )

    return ctx.report()


object_model = build_object_model()
