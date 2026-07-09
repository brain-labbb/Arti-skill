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
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


SILVER = Material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
DARK_STEEL = Material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
ROPE_BLACK = Material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))

# Sheave vertical positions (3 coaxial sheaves)
SHEAVE_Z = [0.065, 0.000, -0.065]


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
    """One continuous bent rod for a swivel hook; sign=+1 for top."""
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
    """Torus ring for the becket eye at the bottom neck.

    Lathe creates around Z (hole along Z), then rotated so hole is along Y
    (parallel to sheave axles). The ring stands vertically in the XZ plane.
    """
    R_major = 0.009
    r_tube = 0.0028
    profile = []
    for i in range(25):
        a = 2.0 * math.pi * i / 24
        profile.append((R_major + r_tube * math.cos(a), r_tube * math.sin(a)))
    geom = LatheGeometry(profile, segments=48, closed=True)
    # Rotate so ring lies in XZ plane with hole along Y.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _rope_mesh():
    """Continuous rope through three sheaves, ending in a compact side coil."""
    pts: list[tuple[float, float, float]] = []
    y = 0.006
    rr = 0.029  # wrap radius slightly outside groove valley

    # Free upper lead from left before the top sheave.
    # Keep the approach below the top neck (neck bottom at z=0.092).
    pts.extend([
        (-0.160, y, 0.118),
        (-0.125, y, 0.108),
        (-0.085, y, 0.092),
        (-0.055, y, 0.082),
        (-0.038, y, 0.076),
    ])

    # Wrap sheave_0 (top, z=0.065) from upper-left to lower-right.
    cx, cz = 0.0, SHEAVE_Z[0]
    for deg in range(170, -31, -18):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Drop on right side to sheave_1.
    pts.extend([(0.028, y, 0.028), (0.028, y, 0.018)])

    # Wrap sheave_1 (middle, z=0.000) counterclockwise from right to left.
    cx, cz = 0.0, SHEAVE_Z[1]
    for deg in range(25, 201, 16):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Drop on left side to sheave_2.
    pts.extend([(-0.028, y, -0.028), (-0.028, y, -0.042)])

    # Wrap sheave_2 (bottom, z=-0.065) clockwise from upper-left to lower-right.
    cx, cz = 0.0, SHEAVE_Z[2]
    for deg in range(155, -51, -18):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Exit to the right and curve down toward coil area.
    pts.extend([
        (0.038, y, -0.090),
        (0.058, y, -0.102),
        (0.068, y, -0.118),
    ])

    # Compact flat spiral coil to the right side, below the block.
    coil_cx, coil_cz = 0.060, -0.138
    turns = 3.0
    samples = 80
    for i in range(samples):
        t = i / (samples - 1)
        angle = math.radians(10) + turns * 2.0 * math.pi * t
        rad = 0.022 - 0.016 * t
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
        name="triple_sheave_becket_pulley",
        meta={
            "run_notes": (
                "Triple-sheave becket block for high-ratio (7:1) block-and-tackle. "
                "Three coaxial sheaves via loop on a widened double-block frame with "
                "a fixed becket eye at the bottom neck for the standing end. "
                "Compatibility probe: N=3 multiplicity and becket attachment coexist "
                "without clearance conflicts."
            )
        },
    )

    model.material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
    model.material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))

    # ── Frame ──────────────────────────────────────────────────────
    frame = model.part("frame")

    # Widened cheek plates for 3-sheave stack plus becket area.
    outer_plate = _plate_mesh("outer_cheek_plate", 0.062, 0.210, 0.0034)
    inner_plate = _plate_mesh("inner_dark_plate", 0.056, 0.200, 0.0024)
    for y_pos, mat, mesh, label in [
        (0.0225, "brushed_silver", outer_plate, "front_outer_plate"),
        (-0.0225, "brushed_silver", outer_plate, "rear_outer_plate"),
        (0.0150, "dark_inner_steel", inner_plate, "front_inner_plate"),
        (-0.0150, "dark_inner_steel", inner_plate, "rear_inner_plate"),
    ]:
        frame.visual(mesh, origin=Origin(xyz=(0.0, y_pos, 0.0)), material=mat, name=label)

    # Layered dark spacer rails visible between the silver side plates.
    for x in (-0.029, 0.029):
        frame.visual(
            Box((0.004, 0.036, 0.165)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="dark_inner_steel",
            name=f"side_spacer_{'neg' if x < 0 else 'pos'}",
        )

    # Top and bottom necks (bridges into inner plates; bottom stays above rope wrap peak).
    for z, label in [(0.112, "top_neck"), (-0.112, "bottom_neck")]:
        frame.visual(
            Box((0.014, 0.040, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="brushed_silver",
            name=label,
        )

    # Axles for each sheave plus structural rivets tying the stacked plates.
    axle_and_rivet_specs = [
        (SHEAVE_Z[0], 0.0040, "sheave_0_axle"),
        (SHEAVE_Z[1], 0.0040, "sheave_1_axle"),
        (SHEAVE_Z[2], 0.0040, "sheave_2_axle"),
        (0.095, 0.0032, "top_rivet"),
        (-0.095, 0.0032, "bottom_rivet"),
    ]
    for z, radius, label in axle_and_rivet_specs:
        frame.visual(
            Cylinder(radius=radius, length=0.053),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name=label,
        )
        for y_head in (0.0265, -0.0265):
            frame.visual(
                Cylinder(radius=radius * 1.55, length=0.0022),
                origin=Origin(xyz=(0.0, y_head, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="brushed_silver",
                name=f"{label}_{'front' if y_head > 0 else 'rear'}_head",
            )

    # Top swivel collar for the hook.
    frame.visual(
        Cylinder(radius=0.0068, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material="brushed_silver",
        name="top_collar",
    )

    # Fixed becket eye at the bottom neck: stem + ring for standing end attachment.
    frame.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material="brushed_silver",
        name="becket_stem",
    )
    frame.visual(
        _becket_eye_mesh("becket_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material="brushed_silver",
        name="becket_eye",
    )

    # ── Sheaves (N=3, loop-emitted) ───────────────────────────────
    for i, z in enumerate(SHEAVE_Z):
        sheave = model.part(f"sheave_{i}")
        sheave.visual(
            _sheave_mesh(f"sheave_{i}_grooved_wheel"),
            material="dark_inner_steel",
            name="grooved_wheel",
        )
        sheave.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, 0.0088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name="front_hub_washer",
        )
        sheave.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, -0.0088, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name="rear_hub_washer",
        )
        model.articulation(
            f"frame_to_sheave_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=sheave,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    # ── Top hook ──────────────────────────────────────────────────
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
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    # ── Rope ──────────────────────────────────────────────────────
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
    sheave_0 = object_model.get_part("sheave_0")
    sheave_1 = object_model.get_part("sheave_1")
    sheave_2 = object_model.get_part("sheave_2")
    top_hook = object_model.get_part("top_hook")
    rope = object_model.get_part("rope")

    joint_0 = object_model.get_articulation("frame_to_sheave_0")
    joint_1 = object_model.get_articulation("frame_to_sheave_1")
    joint_2 = object_model.get_articulation("frame_to_sheave_2")
    top_swivel = object_model.get_articulation("frame_to_top_hook")

    # Part count: frame + 3 sheaves + top_hook + rope = 6
    ctx.check("has frame, 3 sheaves, hook, and rope", len(object_model.parts) == 6)

    # All three sheaves captured between cheek plates.
    for i, sh in enumerate([sheave_0, sheave_1, sheave_2]):
        ctx.expect_within(
            sh, frame, axes="xy", margin=0.002,
            name=f"sheave_{i} captured between side plates",
        )

    # Three sheaves vertically stacked with proper spacing.
    ctx.expect_origin_gap(
        sheave_0, sheave_1, axis="z", min_gap=0.055, max_gap=0.075,
        name="sheave_0 above sheave_1",
    )
    ctx.expect_origin_gap(
        sheave_1, sheave_2, axis="z", min_gap=0.055, max_gap=0.075,
        name="sheave_1 above sheave_2",
    )

    # Top hook still swivels above the block.
    ctx.expect_origin_gap(
        top_hook, frame, axis="z", min_gap=0.115, max_gap=0.135,
        name="top swivel mounted above widened block",
    )

    # Becket eye clearance: the fixed becket ring must clear the bottom sheave.
    ctx.expect_gap(
        sheave_2, frame, axis="z", min_gap=0.025,
        negative_elem="becket_eye",
        name="becket eye clears bottom sheave stack",
    )

    # Rope spans the full triple-sheave stack.
    ctx.expect_overlap(
        rope, frame, axes="z", min_overlap=0.18,
        name="rope spans triple sheave stack",
    )

    # Articulated pose: all sheaves spin, hook swivels, everything stays captured.
    rest_0_pos = ctx.part_world_position(sheave_0)
    with ctx.pose({joint_0: 1.2, joint_1: -0.9, joint_2: 0.7, top_swivel: 0.8}):
        moved_0_pos = ctx.part_world_position(sheave_0)
        for i, sh in enumerate([sheave_0, sheave_1, sheave_2]):
            ctx.expect_within(
                sh, frame, axes="xy", margin=0.002,
                name=f"rotated sheave_{i} remains captured",
            )
        ctx.expect_origin_gap(
            top_hook, frame, axis="z", min_gap=0.115, max_gap=0.135,
            name="top hook swivels on same mount",
        )

    ctx.check(
        "sheave_0 spin keeps axle center fixed",
        rest_0_pos is not None
        and moved_0_pos is not None
        and abs(rest_0_pos[2] - moved_0_pos[2]) < 1e-6,
        details=f"rest={rest_0_pos}, moved={moved_0_pos}",
    )

    return ctx.report()


object_model = build_object_model()
