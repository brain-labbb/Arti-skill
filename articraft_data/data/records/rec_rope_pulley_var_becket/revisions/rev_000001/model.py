from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


SILVER = Material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
DARK_STEEL = Material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
ROPE_BLACK = Material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))


def _plate_mesh(name: str, width: float, height: float, thickness: float):
    """Rounded vertical cheek plate, extruded in the pulley thickness direction."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius=width * 0.48, corner_segments=12),
        thickness,
        cap=True,
        center=True,
    )
    # ExtrudeGeometry is along local Z; rotate so the plate thickness is along Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _sheave_mesh(name: str):
    """Concave rim sheave with a through bore, authored around local Z."""
    half_w = 0.0075
    profile = [
        (0.0058, -half_w),
        (0.0210, -half_w),
        (0.0265, -0.0055),
        (0.0225, 0.0000),  # rope groove valley
        (0.0265, 0.0055),
        (0.0210, half_w),
        (0.0058, half_w),
    ]
    geom = LatheGeometry(profile, segments=72, closed=True)
    # Rotate local lathe axis from Z into Y so the wheel spins on the block axle.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _hook_mesh(name: str, sign: float):
    """One continuous bent rod for a swivel hook; sign=+1 for top, -1 for bottom."""
    # Local Z points outward from the pulley block for both hooks after sign.
    pts = [
        (0.000, 0.000, sign * 0.000),
        (0.000, 0.000, sign * 0.010),
        (-0.004, 0.000, sign * 0.017),
        (-0.014, 0.000, sign * 0.025),
        (-0.015, 0.000, sign * 0.041),
        (-0.004, 0.000, sign * 0.052),
        (0.013, 0.000, sign * 0.047),
        (0.017, 0.000, sign * 0.031),
        (0.008, 0.000, sign * 0.022),
        (0.003, 0.000, sign * 0.025),
    ]
    geom = tube_from_spline_points(
        pts,
        radius=0.0038,
        samples_per_segment=8,
        radial_segments=18,
        closed_spline=False,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _becket_eye_mesh(name: str):
    """Small fixed torus ring serving as the becket anchor eye for the standing rope end."""
    # Sized to the rope diameter: inner opening ~0.006 m, tube ~0.0025 m radius.
    geom = TorusGeometry(radius=0.008, tube=0.0025, radial_segments=18, tubular_segments=48)
    return mesh_from_geometry(geom, name)


def _rope_mesh():
    """Rope anchored at the becket eye, routed through both sheaves, ending in a side coil."""
    pts: list[tuple[float, float, float]] = []
    # Rope sits in the front half of the sheave groove, outside the cheek plates.
    y = 0.006

    # Standing end anchored near the becket eye at the bottom of the block.
    # Route outside the frame neck/strap to avoid interpenetration.
    pts.extend(
        [
            (0.012, y, -0.105),
            (0.016, y, -0.095),
            (0.020, y, -0.082),
            (0.026, y, -0.065),
        ]
    )

    # Approach and wrap around the lower sheave.
    cx, cz, rr = 0.0, -0.035, 0.029
    for deg in range(-55, 215, 16):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Rise between the wheels on the opposite side.
    pts.extend([(-0.028, y, -0.012), (-0.028, y, 0.006)])

    # Wrap around the upper sheave and exit.
    cx, cz = 0.0, 0.035
    for deg in range(215, -145, -18):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Free lead exiting to the coil.
    pts.extend([(-0.055, y, 0.045), (-0.093, y, 0.055), (-0.118, y, 0.054)])

    # A flat spiral coil to the side, still one continuous rope path.
    coil_cx, coil_cz = -0.143, 0.051
    turns = 3.25
    samples = 92
    for i in range(samples):
        t = i / (samples - 1)
        angle = math.radians(-8) + turns * 2.0 * math.pi * t
        rad = 0.027 - 0.021 * t
        pts.append((coil_cx + rad * math.cos(angle), y, coil_cz + rad * math.sin(angle)))

    geom = tube_from_spline_points(
        pts,
        radius=0.0031,
        samples_per_segment=3,
        radial_segments=14,
        closed_spline=False,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, "routed_rope")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_double_rope_pulley",
        meta={
            "run_notes": (
                "Becket-block variant of the compact double rope pulley: the bottom "
                "swivel hook is replaced by a fixed becket eye (torus ring on a tang "
                "strap) that anchors the standing end of the rope for block-and-tackle "
                "mechanical advantage. Both sheaves remain CONTINUOUS and the top swivel "
                "hook is preserved. No category conflict — still a rope pulley."
            )
        },
    )

    model.material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
    model.material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))

    frame = model.part("frame")

    outer_plate = _plate_mesh("outer_cheek_plate", 0.060, 0.154, 0.0034)
    inner_plate = _plate_mesh("inner_dark_plate", 0.054, 0.145, 0.0024)
    for y, mat, mesh, label in [
        (0.0215, "brushed_silver", outer_plate, "front_outer_plate"),
        (-0.0215, "brushed_silver", outer_plate, "rear_outer_plate"),
        (0.0140, "dark_inner_steel", inner_plate, "front_inner_plate"),
        (-0.0140, "dark_inner_steel", inner_plate, "rear_inner_plate"),
    ]:
        frame.visual(mesh, origin=Origin(xyz=(0.0, y, 0.0)), material=mat, name=label)

    # Layered dark spacer rails visible between the silver side plates.
    for x in (-0.028, 0.028):
        frame.visual(
            Box((0.004, 0.034, 0.118)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="dark_inner_steel",
            name=f"side_spacer_{'neg' if x < 0 else 'pos'}",
        )

    for z, label in [(0.083, "top_neck"), (-0.083, "bottom_neck")]:
        frame.visual(
            Box((0.014, 0.038, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="brushed_silver",
            name=label,
        )

    # Becket strap: thin tang extending below the bottom neck to carry the becket eye.
    frame.visual(
        Box((0.012, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.096)),
        material="brushed_silver",
        name="becket_strap",
    )

    # Axles and rivet/collar heads tie the stacked side plates into one block.
    for z, radius, label in [
        (0.035, 0.0040, "upper_axle"),
        (-0.035, 0.0040, "lower_axle"),
        (0.073, 0.0032, "top_rivet"),
        (-0.073, 0.0032, "bottom_rivet"),
        (0.000, 0.0028, "center_rivet"),
    ]:
        frame.visual(
            Cylinder(radius=radius, length=0.051),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name=label,
        )
        for y in (0.0255, -0.0255):
            frame.visual(
                Cylinder(radius=radius * 1.55, length=0.0022),
                origin=Origin(xyz=(0.0, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="brushed_silver",
                name=f"{label}_{'front' if y > 0 else 'rear'}_head",
            )

    # Top swivel collar; the top hook rotates about its vertical shank.
    frame.visual(
        Cylinder(radius=0.0068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material="brushed_silver",
        name="top_collar",
    )

    # Fixed becket eye at the bottom of the becket strap for anchoring the standing rope end.
    frame.visual(
        _becket_eye_mesh("becket_eye_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material="brushed_silver",
        name="becket_eye",
    )

    for z, label in [(0.035, "upper_sheave"), (-0.035, "lower_sheave")]:
        wheel = model.part(label)
        wheel.visual(_sheave_mesh(f"{label}_grooved_wheel"), material="dark_inner_steel", name="grooved_wheel")
        wheel.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, 0.0088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name="front_hub_washer",
        )
        wheel.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, -0.0088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name="rear_hub_washer",
        )
        model.articulation(
            f"frame_to_{label}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    top_hook = model.part("top_hook")
    top_hook.visual(_hook_mesh("top_hook_body", 1.0), material="brushed_silver", name="hook_body")
    top_hook.visual(
        Cylinder(radius=0.0034, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material="brushed_silver",
        name="swivel_shank",
    )
    model.articulation(
        "frame_to_top_hook",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=top_hook,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    rope = model.part("rope")
    rope.visual(_rope_mesh(), material="black_braided_rope", name="continuous_rope")
    model.articulation(
        "frame_to_rope",
        ArticulationType.FIXED,
        parent=frame,
        child=rope,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sheave")
    lower = object_model.get_part("lower_sheave")
    top_hook = object_model.get_part("top_hook")
    rope = object_model.get_part("rope")
    upper_joint = object_model.get_articulation("frame_to_upper_sheave")
    lower_joint = object_model.get_articulation("frame_to_lower_sheave")
    top_swivel = object_model.get_articulation("frame_to_top_hook")

    ctx.check("has frame, sheaves, top hook, and rope", len(object_model.parts) == 5)
    ctx.expect_within(upper, frame, axes="xy", margin=0.002, name="upper sheave captured between side plates")
    ctx.expect_within(lower, frame, axes="xy", margin=0.002, name="lower sheave captured between side plates")
    ctx.expect_origin_gap(upper, lower, axis="z", min_gap=0.060, max_gap=0.080, name="two sheaves vertically stacked")
    ctx.expect_origin_gap(top_hook, frame, axis="z", min_gap=0.090, max_gap=0.100, name="top swivel mounted above block")
    ctx.expect_overlap(rope, frame, axes="z", min_overlap=0.11, name="rope spans both pulley wheels")

    # Becket eye is rigidly attached to the frame at the bottom neck region.
    becket_visual = frame.get_visual("becket_eye")
    ctx.check(
        "becket_eye exists on frame",
        becket_visual is not None,
        details="Fixed becket eye must be present at the bottom of the frame for standing rope anchor.",
    )
    # The rope standing end is anchored near the becket eye at the bottom of the block.
    ctx.expect_overlap(rope, frame, axes="xy", min_overlap=0.003, name="rope anchored near becket eye in xy footprint")

    rest_upper_pos = ctx.part_world_position(upper)
    with ctx.pose({upper_joint: 1.2, lower_joint: -0.9, top_swivel: 0.8}):
        moved_upper_pos = ctx.part_world_position(upper)
        ctx.expect_within(upper, frame, axes="xy", margin=0.002, name="rotated upper sheave remains captured")
        ctx.expect_within(lower, frame, axes="xy", margin=0.002, name="rotated lower sheave remains captured")
        ctx.expect_origin_gap(top_hook, frame, axis="z", min_gap=0.090, max_gap=0.100, name="top hook swivels on same mount")

    ctx.check(
        "sheave spin keeps axle center fixed",
        rest_upper_pos is not None and moved_upper_pos is not None and abs(rest_upper_pos[2] - moved_upper_pos[2]) < 1e-6,
        details=f"rest={rest_upper_pos}, moved={moved_upper_pos}",
    )

    return ctx.report()


object_model = build_object_model()
