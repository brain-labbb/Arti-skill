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
        name="backpacking_pole_chair",
        meta={
            "source_image": "picture/Camping_Outdoor Gear/Camp chair/001.png",
            "asset_category": "Camping_Outdoor Gear",
            "asset_subcategory": "Camp chair",
            "description": "Ultralight backpacking pole-and-hub camp chair with shock-corded poles, blue ripstop sling, and a low-profile folding frame.",
        },
    )

    # ── Materials ──────────────────────────────────────────────────────
    blue_ripstop = model.material("blue_ripstop_sling", rgba=(0.10, 0.22, 0.52, 1.0))
    gray_fabric = model.material("charcoal_gray_center", rgba=(0.27, 0.29, 0.27, 1.0))
    dark_nylon = model.material("dark_nylon_trim", rgba=(0.04, 0.04, 0.05, 1.0))
    pole_metal = model.material("anodized_aluminum_pole", rgba=(0.52, 0.52, 0.50, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.015, 1.0))
    rubber = model.material("rubber_tip", rgba=(0.06, 0.06, 0.06, 1.0))
    hub_body = model.material("hub_connector", rgba=(0.08, 0.08, 0.09, 1.0))
    white_trim = model.material("white_logo_trim", rgba=(0.90, 0.90, 0.88, 1.0))

    # ── CHAIR FRAME: pole-and-hub skeleton ────────────────────────────
    frame = model.part("chair_frame")

    # Central hub block where all four shock-corded poles converge.
    # Sits low on the ground; rubber pads underneath provide grip.
    hub_cx, hub_cy, hub_cz = 0.0, -0.02, 0.045
    _add_box(frame, "central_hub", (0.09, 0.09, 0.038), (hub_cx, hub_cy, hub_cz), hub_body)

    # Four rubber foot pads under the hub corners
    pad_dz = -0.012
    for i, (dx, dy) in enumerate(((-0.032, -0.032), (0.032, -0.032), (-0.032, 0.032), (0.032, 0.032))):
        _add_ball(frame, f"hub_foot_pad_{i}", (hub_cx + dx, hub_cy + dy, hub_cz + pad_dz), 0.010, rubber)

    # Pole parameters (thin shock-corded aluminum segments)
    pole_r = 0.005       # 5 mm pole radius
    ball_r = 0.008       # ball end caps at the hub junction

    # Hub top surface where poles originate
    hub_top = (hub_cx, hub_cy, hub_cz + 0.019)

    # Pole tip positions (where sling corners attach)
    # Front pair: low seat height (~0.30 m). Rear pair: taller for back support (~0.62 m).
    tip_fl = (-0.28, -0.26, 0.30)
    tip_fr = ( 0.28, -0.26, 0.30)
    tip_rl = (-0.24,  0.18, 0.62)
    tip_rr = ( 0.24,  0.18, 0.62)

    pole_tips = {"fl": tip_fl, "fr": tip_fr, "rl": tip_rl, "rr": tip_rr}

    # Four angled poles radiating from the central hub to seat/back corners
    for key, tip in pole_tips.items():
        _add_tube(frame, f"pole_{key}", hub_top, tip, pole_r, pole_metal)
        # Ball end cap at hub (shock-cord anchor)
        _add_ball(frame, f"pole_{key}_hub_ball", hub_top, ball_r, black_plastic)
        # Pole tip cap (where sling pocket clips on)
        _add_ball(frame, f"pole_{key}_tip_cap", tip, 0.006, black_plastic)

    # Structural cross-rails connecting pole tips for sling support
    _add_tube(frame, "seat_front_rail", tip_fl, tip_fr, pole_r, pole_metal)
    _add_tube(frame, "back_top_rail", tip_rl, tip_rr, pole_r, pole_metal)

    # Pivot mount for the folding spreader brace. The spreader connects the
    # two front poles at roughly their mid-height to keep the front pair splayed.
    # Position the pivot at the midpoint of the front poles.
    # At z≈0.15 the front poles are at x≈±0.10, y≈-0.11.
    pivot_xyz = (0.0, -0.11, 0.162)
    _add_box(frame, "pivot_mount", (0.022, 0.026, 0.040), pivot_xyz, hub_body)
    _add_tube(frame, "pivot_strut", (hub_cx, hub_cy - 0.030, hub_cz + 0.019), pivot_xyz, 0.004, pole_metal)
    # Pivot pin oriented along Y (rotation axis), offset above the spreader bar
    _add_tube(frame, "pivot_pin", (0.0, -0.13, 0.162), (0.0, -0.09, 0.162), 0.004, black_plastic)

    # ── SLING SEAT: hangs from the four pole tips ─────────────────────
    seat_z = 0.28  # very low seat height, Helinox-style

    # Blue ripstop side panels
    _add_box(frame, "seat_blue_left", (0.14, 0.44, 0.010), (-0.20, -0.04, seat_z), blue_ripstop)
    _add_box(frame, "seat_blue_right", (0.14, 0.44, 0.010), (0.20, -0.04, seat_z), blue_ripstop)
    # Gray center panel (preserved name)
    _add_box(frame, "seat_gray_center", (0.24, 0.44, 0.008), (0.0, -0.04, seat_z + 0.002), gray_fabric)

    # Corner pockets where sling clips onto front pole tips
    for key in ("fl", "fr"):
        tx, ty, tz = pole_tips[key]
        _add_box(frame, f"seat_pocket_{key}", (0.04, 0.04, 0.028), (tx, ty, tz - 0.008), dark_nylon)

    # Front edge hem
    _add_box(frame, "seat_front_hem", (0.56, 0.024, 0.014), (0.0, -0.26, seat_z + 0.004), dark_nylon)
    # Rear edge hem
    _add_box(frame, "seat_rear_hem", (0.50, 0.020, 0.012), (0.0, 0.18, seat_z + 0.004), dark_nylon)

    # ── SLING BACK: hangs from rear pole tips up to top rail ──────────
    back_roll = -0.20  # reclined lean
    back_cz = 0.46

    # Blue ripstop outer back panel
    _add_box(frame, "back_blue_panel", (0.46, 0.012, 0.30), (0.0, 0.17, back_cz), blue_ripstop, rpy=(back_roll, 0.0, 0.0))
    # Gray center panel (preserved name)
    _add_box(frame, "back_gray_center", (0.24, 0.010, 0.28), (0.0, 0.17, back_cz + 0.002), gray_fabric, rpy=(back_roll, 0.0, 0.0))

    # Corner pockets where sling clips onto rear pole tips
    for key in ("rl", "rr"):
        tx, ty, tz = pole_tips[key]
        _add_box(frame, f"back_pocket_{key}", (0.04, 0.04, 0.028), (tx, ty, tz - 0.008), dark_nylon)

    # Back top hem and logo
    _add_box(frame, "back_top_hem", (0.46, 0.022, 0.014), (0.0, 0.17, 0.61), dark_nylon, rpy=(back_roll, 0.0, 0.0))
    _add_box(frame, "back_logo", (0.060, 0.014, 0.030), (0.0, 0.17, 0.52), white_trim, rpy=(back_roll, 0.0, 0.0))

    # ── FRONT CROSS BRACE: folding spreader bar ───────────────────────
    # A thin cross-tube that pivots to collapse the front pole pair for packing.
    # Local origin is at the pivot point; tube extends horizontally between
    # the two front poles at their mid-height.
    front_brace = model.part("front_cross_brace")
    _add_tube(front_brace, "front_spreader_bar", (-0.098, 0.0, 0.0), (0.098, 0.0, 0.0), pole_r, pole_metal)
    _add_tube(front_brace, "front_pivot_collar", (0.0, -0.008, 0.0), (0.0, 0.008, 0.0), 0.007, black_plastic)
    _add_ball(front_brace, "front_brace_end_0", (-0.098, 0.0, 0.0), 0.006, black_plastic)
    _add_ball(front_brace, "front_brace_end_1", (0.098, 0.0, 0.0), 0.006, black_plastic)

    # ── ARTICULATION: pole-frame fold joint ───────────────────────────
    model.articulation(
        "frame_to_front_cross",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_brace,
        origin=Origin(xyz=(0.0, -0.11, 0.162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("chair_frame")
    front = object_model.get_part("front_cross_brace")
    front_joint = object_model.get_articulation("frame_to_front_cross")

    # ── Allow intentional overlap: pivot pin captured inside collar ────
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_pin",
        elem_b="front_pivot_collar",
        reason="The folding spreader bar collar is intentionally captured around the fixed pivot pin.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_mount",
        elem_b="front_pivot_collar",
        reason="The pivot mount block intentionally nests the rotating collar at the fold joint.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_mount",
        elem_b="front_spreader_bar",
        reason="The spreader bar passes through the pivot mount at rest; the mount captures the bar for folding.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_pin",
        elem_b="front_spreader_bar",
        reason="The pivot pin crosses the spreader bar at the rotation joint; both are captured inside the collar.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pivot_strut",
        elem_b="front_pivot_collar",
        reason="The pivot strut terminates at the collar junction; local embedding represents the strut-to-collar structural connection.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pole_fl",
        elem_b="front_spreader_bar",
        reason="The spreader bar clips onto the front-left pole at a T-joint connection; slight local embedding represents the snap-fit clip.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pole_fr",
        elem_b="front_spreader_bar",
        reason="The spreader bar clips onto the front-right pole at a T-joint connection; slight local embedding represents the snap-fit clip.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pole_fl",
        elem_b="front_brace_end_0",
        reason="The spreader bar end cap intentionally clips onto the front-left pole at a T-joint connection.",
    )
    ctx.allow_overlap(
        frame,
        front,
        elem_a="pole_fr",
        elem_b="front_brace_end_1",
        reason="The spreader bar end cap intentionally clips onto the front-right pole at a T-joint connection.",
    )

    # ── Provenance metadata ──────────────────────────────────────────
    ctx.check(
        "provenance metadata preserved",
        object_model.meta.get("source_image") == "picture/Camping_Outdoor Gear/Camp chair/001.png"
        and object_model.meta.get("asset_category") == "Camping_Outdoor Gear"
        and object_model.meta.get("asset_subcategory") == "Camp chair",
        details=f"metadata={object_model.meta}",
    )

    # ── Pole-and-hub frame structure ─────────────────────────────────
    # Verify the central hub and four radiating poles exist on the frame
    hub_visual = frame.get_visual("central_hub")
    ctx.check(
        "central_hub exists on chair_frame",
        hub_visual is not None,
        details="chair_frame must have a central_hub visual for the pole-and-hub backpacking frame",
    )

    pole_names = ["pole_fl", "pole_fr", "pole_rl", "pole_rr"]
    for pn in pole_names:
        v = frame.get_visual(pn)
        ctx.check(
            f"pole {pn} exists on chair_frame",
            v is not None,
            details=f"chair_frame must have thin pole {pn} radiating from the hub",
        )

    # Seat and back gray center panels preserved
    ctx.check(
        "seat_gray_center preserved",
        frame.get_visual("seat_gray_center") is not None,
        details="seat_gray_center visual must exist on chair_frame",
    )
    ctx.check(
        "back_gray_center preserved",
        frame.get_visual("back_gray_center") is not None,
        details="back_gray_center visual must exist on chair_frame",
    )

    # ── Low seat height (backpacking chair is dramatically lower) ────
    seat_aabb = ctx.part_element_world_aabb(frame, elem="seat_gray_center")
    ctx.check(
        "seat height is low (backpacking style)",
        seat_aabb is not None and seat_aabb[0][2] < 0.35,
        details=f"seat_gray_center min_z={seat_aabb[0][2] if seat_aabb else None}, expected < 0.35 m",
    )

    # ── Front cross-brace pivot retained and folding ─────────────────
    ctx.expect_overlap(
        frame, front,
        axes="yz",
        min_overlap=0.006,
        elem_a="pivot_pin",
        elem_b="front_pivot_collar",
        name="spreader collar retained on pivot pin",
    )

    closed_aabb = ctx.part_world_aabb(front)
    with ctx.pose({front_joint: 0.50}):
        folded_aabb = ctx.part_world_aabb(front)
        ctx.expect_overlap(
            frame, front,
            axes="yz",
            min_overlap=0.008,
            elem_a="pivot_pin",
            elem_b="front_pivot_collar",
            name="pivot still captured while folding",
        )

    ctx.check(
        "front_cross_brace folds via frame_to_front_cross",
        closed_aabb is not None
        and folded_aabb is not None
        and abs((folded_aabb[1][2] - folded_aabb[0][2]) - (closed_aabb[1][2] - closed_aabb[0][2])) > 0.015,
        details=f"rest={closed_aabb}, folded={folded_aabb}",
    )

    # ── No arm rests or heavy frame remnants ─────────────────────────
    frame_visual_names = {v.name for v in frame.visuals}
    for forbidden in ("arm_rest_top_0", "arm_rest_top_1", "front_leg_0", "front_leg_1",
                       "rear_back_post_0", "rear_back_post_1", "cup_holder_ring"):
        ctx.check(
            f"no {forbidden} on pole-and-hub frame",
            forbidden not in frame_visual_names,
            details=f"heavy quad-leg or armrest visual {forbidden} must not exist on the backpacking frame",
        )

    return ctx.report()


object_model = build_object_model()
