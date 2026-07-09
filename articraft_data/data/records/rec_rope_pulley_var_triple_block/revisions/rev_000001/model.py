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


# --- Materials ---------------------------------------------------------------

SILVER = Material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
DARK_STEEL = Material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
ROPE_BLACK = Material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))
ANODIZED_BLACK = Material("black_anodized_sheave", rgba=(0.06, 0.06, 0.09, 1.0))

# --- Sheave layout -----------------------------------------------------------

N_SHEAVES = 3
SHEAVE_Z_OFFSETS = [-0.044, 0.0, 0.044]  # bottom, middle, top

# Widened plate Y positions for triple-sheave block
OUTER_PLATE_Y = 0.0240
INNER_PLATE_Y = 0.0158
AXLE_LENGTH = 0.057
RIVET_HEAD_Y = 0.0280


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
        (0.0170, -half_w),
        (0.0210, -0.0050),
        (0.0180, 0.0000),  # rope groove valley
        (0.0210, 0.0050),
        (0.0170, half_w),
        (0.0058, half_w),
    ]
    geom = LatheGeometry(profile, segments=72, closed=True)
    # Rotate local lathe axis from Z into Y so the wheel spins on the block axle.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _hook_mesh(name: str, sign: float):
    """One continuous bent rod for a swivel hook; sign=+1 for top, -1 for bottom."""
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


def _rope_mesh():
    """Continuous rope routed through three sheave grooves, ending in a side coil."""
    pts: list[tuple[float, float, float]] = []
    y = 0.006  # front half of groove so rope sits in the visible channel

    # Free upper lead with a small loose bend before the top sheave.
    pts.extend([
        (-0.160, y, 0.072),
        (-0.138, y, 0.076),
        (-0.110, y, 0.068),
        (-0.055, y, 0.055),
    ])

    # Wrap around top sheave (sheave_2, z=+0.044) — clockwise from upper-left.
    rr = 0.022
    cx, cz = 0.0, SHEAVE_Z_OFFSETS[2]
    for deg in range(170, -145, -18):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Transition down to middle sheave (right side).
    pts.extend([(0.022, y, 0.018), (0.022, y, 0.008)])

    # Wrap around middle sheave (sheave_1, z=0.0) — counter-clockwise.
    cx, cz = 0.0, SHEAVE_Z_OFFSETS[1]
    for deg in range(35, 220, 16):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Transition down to bottom sheave (left side).
    pts.extend([(-0.020, y, -0.018), (-0.018, y, -0.028)])

    # Wrap around bottom sheave (sheave_0, z=-0.044) — clockwise.
    cx, cz = 0.0, SHEAVE_Z_OFFSETS[0]
    for deg in range(170, -145, -18):
        a = math.radians(deg)
        pts.append((cx + rr * math.cos(a), y, cz + rr * math.sin(a)))

    # Exit to the side coil.
    pts.extend([(-0.056, y, -0.066), (-0.093, y, -0.076), (-0.118, y, -0.075)])

    # A flat spiral coil to the side, still one continuous rope path.
    coil_cx, coil_cz = -0.143, -0.072
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
        name="triple_sheave_rope_pulley",
        meta={
            "run_notes": (
                "Triple-sheave rope pulley block variant: three independently "
                "spinning grooved sheaves stacked between four cheek plates, "
                "with top and bottom swivel hooks and a continuous rope routed "
                "through all three grooves. Suitable as the moving/standing "
                "block in a 3:1 or 6:1 block-and-tackle haul system. "
                "Black-anodized sheave colorway with silver plates."
            )
        },
    )

    model.material("brushed_silver", rgba=(0.78, 0.78, 0.74, 1.0))
    model.material("dark_inner_steel", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("black_braided_rope", rgba=(0.003, 0.003, 0.003, 1.0))
    model.material("black_anodized_sheave", rgba=(0.06, 0.06, 0.09, 1.0))

    # --- Frame (root part) ---------------------------------------------------
    frame = model.part("frame")

    # Four stacked cheek plates: two outer silver, two inner dark.
    outer_plate = _plate_mesh("outer_cheek_plate", 0.062, 0.176, 0.0034)
    inner_plate = _plate_mesh("inner_dark_plate", 0.056, 0.166, 0.0024)
    for y_pos, mat, mesh, label in [
        (OUTER_PLATE_Y, "brushed_silver", outer_plate, "front_outer_plate"),
        (-OUTER_PLATE_Y, "brushed_silver", outer_plate, "rear_outer_plate"),
        (INNER_PLATE_Y, "dark_inner_steel", inner_plate, "front_inner_plate"),
        (-INNER_PLATE_Y, "dark_inner_steel", inner_plate, "rear_inner_plate"),
    ]:
        frame.visual(mesh, origin=Origin(xyz=(0.0, y_pos, 0.0)), material=mat, name=label)

    # Dark spacer rails visible between the silver side plates.
    for x in (-0.029, 0.029):
        frame.visual(
            Box((0.004, 0.038, 0.140)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="dark_inner_steel",
            name=f"side_spacer_{'neg' if x < 0 else 'pos'}",
        )

    # Top and bottom necks connect the block to the hook swivel collars.
    for z, label in [(0.095, "top_neck"), (-0.095, "bottom_neck")]:
        frame.visual(
            Box((0.014, 0.044, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="brushed_silver",
            name=label,
        )

    # Axles (one per sheave) and structural rivets tie the plates together.
    for z, radius, label in [
        (SHEAVE_Z_OFFSETS[0], 0.0040, "sheave_0_axle"),
        (SHEAVE_Z_OFFSETS[1], 0.0040, "sheave_1_axle"),
        (SHEAVE_Z_OFFSETS[2], 0.0040, "sheave_2_axle"),
        (0.085, 0.0032, "top_rivet"),
        (-0.085, 0.0032, "bottom_rivet"),
    ]:
        frame.visual(
            Cylinder(radius=radius, length=AXLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name=label,
        )
        for y_head in (RIVET_HEAD_Y, -RIVET_HEAD_Y):
            frame.visual(
                Cylinder(radius=radius * 1.55, length=0.0022),
                origin=Origin(xyz=(0.0, y_head, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="brushed_silver",
                name=f"{label}_{'front' if y_head > 0 else 'rear'}_head",
            )

    # Top and bottom swivel collars; hooks rotate about their vertical shanks.
    for z, label in [(0.099, "top_collar"), (-0.099, "bottom_collar")]:
        frame.visual(
            Cylinder(radius=0.0068, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="brushed_silver",
            name=label,
        )

    # --- Three sheaves (indexed loop) ----------------------------------------
    for i, z in enumerate(SHEAVE_Z_OFFSETS):
        sheave = model.part(f"sheave_{i}")
        sheave.visual(
            _sheave_mesh(f"sheave_{i}_grooved_wheel"),
            material="black_anodized_sheave",
            name="grooved_wheel",
        )
        sheave.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, 0.0090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_silver",
            name="front_hub_washer",
        )
        sheave.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, -0.0090, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
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

    # --- Top swivel hook -----------------------------------------------------
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
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    # --- Bottom swivel hook --------------------------------------------------
    bottom_hook = model.part("bottom_hook")
    bottom_hook.visual(_hook_mesh("bottom_hook_body", -1.0), material="brushed_silver", name="hook_body")
    bottom_hook.visual(
        Cylinder(radius=0.0034, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material="brushed_silver",
        name="swivel_shank",
    )
    model.articulation(
        "frame_to_bottom_hook",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bottom_hook,
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    # --- Rope (fixed to frame) -----------------------------------------------
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
    sheaves = [object_model.get_part(f"sheave_{i}") for i in range(N_SHEAVES)]
    sheave_joints = [object_model.get_articulation(f"frame_to_sheave_{i}") for i in range(N_SHEAVES)]
    top_hook = object_model.get_part("top_hook")
    bottom_hook = object_model.get_part("bottom_hook")
    rope = object_model.get_part("rope")
    top_swivel = object_model.get_articulation("frame_to_top_hook")
    bottom_swivel = object_model.get_articulation("frame_to_bottom_hook")

    # Part count: frame + 3 sheaves + top_hook + bottom_hook + rope = 7
    ctx.check("has frame, 3 sheaves, 2 hooks, and rope", len(object_model.parts) == 7)

    # All three sheave joints are CONTINUOUS about Y axis
    for i, joint in enumerate(sheave_joints):
        ctx.check(
            f"frame_to_sheave_{i} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"got {joint.articulation_type}",
        )

    # Each sheave is captured between the side plates (within frame XY footprint)
    for i, sheave in enumerate(sheaves):
        ctx.expect_within(
            sheave, frame, axes="xy", margin=0.002,
            name=f"sheave_{i} captured between side plates",
        )

    # Sheaves are vertically stacked with correct spacing
    ctx.expect_origin_gap(
        sheaves[2], sheaves[1], axis="z", min_gap=0.025, max_gap=0.045,
        name="sheave_2 above sheave_1 with correct spacing",
    )
    ctx.expect_origin_gap(
        sheaves[1], sheaves[0], axis="z", min_gap=0.025, max_gap=0.045,
        name="sheave_1 above sheave_0 with correct spacing",
    )

    # Hooks mounted above and below the block
    ctx.expect_origin_gap(
        top_hook, frame, axis="z", min_gap=0.090, max_gap=0.110,
        name="top swivel mounted above block",
    )
    ctx.expect_origin_gap(
        frame, bottom_hook, axis="z", min_gap=0.090, max_gap=0.110,
        name="bottom swivel mounted below block",
    )

    # Rope spans all three pulley wheels
    ctx.expect_overlap(
        rope, frame, axes="z", min_overlap=0.10,
        name="rope spans triple sheave block vertically",
    )

    # Articulated pose: spin all sheaves and swivel hooks
    rest_positions = [ctx.part_world_position(s) for s in sheaves]
    pose_dict = {
        sheave_joints[0]: 1.5,
        sheave_joints[1]: -1.2,
        sheave_joints[2]: 0.9,
        top_swivel: 0.8,
        bottom_swivel: -0.8,
    }
    with ctx.pose(pose_dict):
        for i, sheave in enumerate(sheaves):
            ctx.expect_within(
                sheave, frame, axes="xy", margin=0.002,
                name=f"rotated sheave_{i} remains captured",
            )
        ctx.expect_origin_gap(
            top_hook, frame, axis="z", min_gap=0.090, max_gap=0.110,
            name="top hook swivels on same mount",
        )
        ctx.expect_origin_gap(
            frame, bottom_hook, axis="z", min_gap=0.090, max_gap=0.110,
            name="bottom hook swivels on same mount",
        )

    # Sheave spin keeps axle centers fixed (continuous rotation about Y)
    for i in range(N_SHEAVES):
        moved_pos = ctx.part_world_position(sheaves[i])
        ctx.check(
            f"sheave_{i} spin keeps axle center fixed",
            rest_positions[i] is not None and moved_pos is not None
            and abs(rest_positions[i][2] - moved_pos[2]) < 1e-6,
            details=f"rest={rest_positions[i]}, moved={moved_pos}",
        )

    return ctx.report()


object_model = build_object_model()
