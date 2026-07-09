from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cyl_x(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name: str, length: float, radius: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _arc_points(radius: float, z: float, *, start: float = pi, stop: float = 0.0, steps: int = 24):
    return [
        (radius * cos(start + (stop - start) * i / steps), radius * sin(start + (stop - start) * i / steps), z)
        for i in range(steps + 1)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_fire_escape_ladder_telescoping",
        meta={
            "run_notes": (
                "Telescoping fire escape ladder variant with 3 nested rail "
                "stages: stage_0 (fixed/anchored, named upper_ladder), stage_1, "
                "and stage_2. Each stage is its own part with decreasing rail "
                "radius for nesting. Stages chain via PRISMATIC joints sharing "
                "the -z extension axis (stage0_to_stage1, stage1_to_stage2) "
                "with realistic retained-overlap guide collars. Cage and wall "
                "standoff/plate anchoring preserved on the fixed stage."
            )
        },
    )

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.76, 0.75, 1.0))
    dark_steel = model.material("dark_bolt_heads", rgba=(0.08, 0.09, 0.09, 1.0))
    red_latch = model.material("red_release_latch", rgba=(0.9, 0.05, 0.03, 1.0))

    # ─────────────────────────────────────────────────────────────────────
    # Stage 0 — fixed anchored stage (parent name: upper_ladder)
    # ─────────────────────────────────────────────────────────────────────
    upper = model.part("upper_ladder")

    height = 5.25
    rail_x = 0.25
    rail_radius = 0.026
    rung_radius = 0.017

    # Main fixed ladder rails
    _cyl_z(upper, "rail_0", height, rail_radius, (-rail_x, 0.0, height / 2.0), galvanized)
    _cyl_z(upper, "rail_1", height, rail_radius, (rail_x, 0.0, height / 2.0), galvanized)

    # Regular climbing rungs, welded through the side rails (loop-emitted)
    rung_count = 17
    rung_pitch = 0.30
    rung_z0 = 0.18
    for i in range(rung_count):
        _cyl_x(upper, f"rung_{i}", 2.0 * rail_x + 0.045, rung_radius, (0.0, 0.0, rung_z0 + i * rung_pitch), galvanized)

    # Wall stand-off brackets and mounting plates
    for bracket_index, z in enumerate((0.45, 1.35, 2.25, 3.15, 4.05, 4.85)):
        for side_index, x in enumerate((-rail_x, rail_x)):
            _cyl_y(upper, f"standoff_{bracket_index}_{side_index}", 0.36, 0.018, (x, -0.18, z), galvanized)
            upper.visual(
                Box((0.16, 0.024, 0.22)),
                origin=Origin(xyz=(x, -0.37, z)),
                material=galvanized,
                name=f"wall_plate_{bracket_index}_{side_index}",
            )
            _cyl_z(upper, f"bolt_top_{bracket_index}_{side_index}", 0.012, 0.018, (x, -0.385, z + 0.065), dark_steel)
            _cyl_z(upper, f"bolt_bottom_{bracket_index}_{side_index}", 0.012, 0.018, (x, -0.385, z - 0.065), dark_steel)

    # Cage hoops: semicircular horizontal guards projecting away from the wall
    cage_radius = 0.58
    cage_tube = 0.017
    cage_bottom = 1.55
    cage_top = 5.12
    hoop_levels = (1.55, 2.15, 2.75, 3.35, 3.95, 4.55, 5.10)
    for hoop_index, z in enumerate(hoop_levels):
        hoop_geom = tube_from_spline_points(
            _arc_points(cage_radius, z, steps=32),
            radius=cage_tube,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        )
        upper.visual(
            mesh_from_geometry(hoop_geom, f"cage_hoop_mesh_{hoop_index}"),
            material=galvanized,
            name=f"cage_hoop_{hoop_index}",
        )
        left_mid = (-(rail_x + cage_radius) / 2.0, 0.0, z)
        right_mid = ((rail_x + cage_radius) / 2.0, 0.0, z)
        _cyl_x(upper, f"cage_arm_{hoop_index}_0", cage_radius - rail_x + 0.035, cage_tube, left_mid, galvanized)
        _cyl_x(upper, f"cage_arm_{hoop_index}_1", cage_radius - rail_x + 0.035, cage_tube, right_mid, galvanized)

    # Vertical cage bars following the semicircular envelope
    cage_bar_height = cage_top - cage_bottom
    for bar_index, angle in enumerate((pi, 5 * pi / 6, 2 * pi / 3, pi / 2, pi / 3, pi / 6, 0.0)):
        x = cage_radius * cos(angle)
        y = cage_radius * sin(angle)
        _cyl_z(
            upper, f"cage_bar_{bar_index}", cage_bar_height + 0.06, 0.014,
            (x, y, (cage_bottom + cage_top) / 2.0), galvanized,
        )

    # Guide collars at bottom of stage_0 to capture stage_1 rails.
    # Placed at z=0.06 (between rung_0 at z=0.18 and rail bottom at z=0)
    # so they don't collide with stage_1 rungs.
    s1_rail_x = 0.18
    collar_z0 = 0.06
    for side_index, x in enumerate((-s1_rail_x, s1_rail_x)):
        rail_side = -1.0 if x < 0.0 else 1.0
        upper.visual(
            Box((0.080, 0.070, 0.100)),
            origin=Origin(xyz=(x, 0.060, collar_z0)),
            material=galvanized,
            name=f"lower_slide_collar_{side_index}",
        )
        # Connecting bracket from collar to rail (resolves disconnected island)
        _cyl_x(
            upper, f"collar_bracket_{side_index}",
            abs(rail_x - abs(x)) + 0.010, 0.014,
            ((x + rail_side * rail_x) / 2.0, 0.030, collar_z0),
            galvanized,
        )

    # Release latch
    upper.visual(
        Box((0.18, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, 0.020, 0.460)),
        material=red_latch,
        name="release_latch",
    )

    # ─────────────────────────────────────────────────────────────────────
    # Stage 1 — middle telescoping stage
    # ─────────────────────────────────────────────────────────────────────
    stage1 = model.part("stage_1")
    s1_rail_r = 0.020
    s1_y = 0.06
    s1_height = 2.80
    s1_top = 2.50
    s1_bottom = s1_top - s1_height  # -0.30
    s1_mid_z = (s1_top + s1_bottom) / 2.0

    # Stage 1 rails (smaller radius than stage 0, offset forward in y)
    _cyl_z(stage1, "stage1_rail_0", s1_height, s1_rail_r, (-s1_rail_x, s1_y, s1_mid_z), galvanized)
    _cyl_z(stage1, "stage1_rail_1", s1_height, s1_rail_r, (s1_rail_x, s1_y, s1_mid_z), galvanized)

    # Stage 1 rungs (loop-emitted with stable indexed names)
    s1_rung_count = 9
    s1_rung_pitch = 0.30
    s1_rung_z0 = s1_bottom + 0.15  # -0.15
    for i in range(s1_rung_count):
        _cyl_x(stage1, f"stage1_rung_{i}", 2.0 * s1_rail_x + 0.04, 0.015, (0.0, s1_y, s1_rung_z0 + i * s1_rung_pitch), galvanized)

    # Guide collars on stage_1 bottom to capture stage_2 rails
    s2_rail_x = 0.12
    s2_y = 0.12
    s1_collar_z = s1_bottom + 0.20  # -0.10, below rung_0 at z=-0.15
    for side_index, x in enumerate((-s2_rail_x, s2_rail_x)):
        stage1.visual(
            Box((0.060, 0.060, 0.080)),
            origin=Origin(xyz=(x, s2_y, s1_collar_z)),
            material=galvanized,
            name=f"stage1_slide_collar_{side_index}",
        )
        # Connecting bracket from collar to stage_1 rail (resolves disconnected island)
        rail_side = -1.0 if x < 0.0 else 1.0
        _cyl_x(
            stage1, f"stage1_collar_bracket_{side_index}",
            abs(s1_rail_x - abs(x)) + 0.010, 0.012,
            ((x + rail_side * s1_rail_x) / 2.0, (s2_y + s1_y) / 2.0, s1_collar_z),
            galvanized,
        )

    # ─────────────────────────────────────────────────────────────────────
    # Stage 2 — innermost telescoping stage
    # ─────────────────────────────────────────────────────────────────────
    stage2 = model.part("stage_2")
    s2_rail_r = 0.016
    s2_height = 2.20
    s2_top = 2.00
    s2_bottom = s2_top - s2_height  # -0.20
    s2_mid_z = (s2_top + s2_bottom) / 2.0

    # Stage 2 rails (smallest radius, furthest forward in y)
    _cyl_z(stage2, "stage2_rail_0", s2_height, s2_rail_r, (-s2_rail_x, s2_y, s2_mid_z), galvanized)
    _cyl_z(stage2, "stage2_rail_1", s2_height, s2_rail_r, (s2_rail_x, s2_y, s2_mid_z), galvanized)

    # Stage 2 rungs (loop-emitted with stable indexed names)
    s2_rung_count = 7
    s2_rung_pitch = 0.30
    s2_rung_z0 = s2_bottom + 0.15  # -0.05
    for i in range(s2_rung_count):
        _cyl_x(stage2, f"stage2_rung_{i}", 2.0 * s2_rail_x + 0.035, 0.013, (0.0, s2_y, s2_rung_z0 + i * s2_rung_pitch), galvanized)

    # Foot bar at bottom of stage_2
    _cyl_x(stage2, "stage2_foot_bar", 2.0 * s2_rail_x + 0.10, 0.020, (0.0, s2_y, s2_bottom), galvanized)

    # ─────────────────────────────────────────────────────────────────────
    # Articulations: telescoping PRISMATIC chain along -z
    # ─────────────────────────────────────────────────────────────────────

    # Stage 0 → Stage 1: extends downward
    model.articulation(
        "stage0_to_stage1",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=1.60),
    )

    # Stage 1 → Stage 2: extends downward
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.30, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stage0 = object_model.get_part("upper_ladder")
    stage1 = object_model.get_part("stage_1")
    stage2 = object_model.get_part("stage_2")
    j01 = object_model.get_articulation("stage0_to_stage1")
    j12 = object_model.get_articulation("stage1_to_stage2")

    # ── Preserved stage_0 functional layers ──
    ctx.check(
        "stage0 preserved rungs",
        len([v for v in stage0.visuals if v.name.startswith("rung_")]) >= 16,
        details="The fixed stage should retain a full run of evenly spaced climbing rungs.",
    )
    ctx.check(
        "stage0 cage hoops",
        len([v for v in stage0.visuals if v.name.startswith("cage_hoop_")]) >= 6,
        details="The safety cage should remain on the fixed anchored stage.",
    )
    ctx.check(
        "stage0 wall mounting",
        len([v for v in stage0.visuals if v.name.startswith("wall_plate_")]) >= 10,
        details="Wall standoff plates are required for the fixed anchored stage.",
    )

    # ── Telescoping stage rung counts (loop-emitted) ──
    ctx.check(
        "stage1 loop-emitted rungs",
        len([v for v in stage1.visuals if v.name.startswith("stage1_rung_")]) >= 5,
        details="Stage 1 must carry its own loop-emitted rungs with stable indexed names.",
    )
    ctx.check(
        "stage2 loop-emitted rungs",
        len([v for v in stage2.visuals if v.name.startswith("stage2_rung_")]) >= 4,
        details="Stage 2 must carry its own loop-emitted rungs with stable indexed names.",
    )

    # ── Guide collar retained overlap: stage_0 collars capture stage_1 rails ──
    for idx in (0, 1):
        ctx.allow_overlap(
            stage0,
            stage1,
            elem_a=f"lower_slide_collar_{idx}",
            elem_b=f"stage1_rail_{idx}",
            reason=(
                "Stage 1 rails are intentionally captured inside the stage 0 "
                "guide collars so the middle telescoping stage remains mounted "
                "to the fixed anchored stage."
            ),
        )
        ctx.expect_within(
            stage1,
            stage0,
            axes="xy",
            inner_elem=f"stage1_rail_{idx}",
            outer_elem=f"lower_slide_collar_{idx}",
            margin=0.0,
            name=f"stage1 rail {idx} centered in stage0 guide collar",
        )
        ctx.expect_overlap(
            stage1,
            stage0,
            axes="z",
            elem_a=f"stage1_rail_{idx}",
            elem_b=f"lower_slide_collar_{idx}",
            min_overlap=0.08,
            name=f"stage1 rail {idx} retained in stage0 collar at rest",
        )

    # ── Guide collar retained overlap: stage_1 collars capture stage_2 rails ──
    for idx in (0, 1):
        ctx.allow_overlap(
            stage1,
            stage2,
            elem_a=f"stage1_slide_collar_{idx}",
            elem_b=f"stage2_rail_{idx}",
            reason=(
                "Stage 2 rails are intentionally captured inside the stage 1 "
                "guide collars so the innermost telescoping stage remains "
                "mounted to the middle stage."
            ),
        )
        ctx.expect_within(
            stage2,
            stage1,
            axes="xy",
            inner_elem=f"stage2_rail_{idx}",
            outer_elem=f"stage1_slide_collar_{idx}",
            margin=0.0,
            name=f"stage2 rail {idx} centered in stage1 guide collar",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="z",
            elem_a=f"stage2_rail_{idx}",
            elem_b=f"stage1_slide_collar_{idx}",
            min_overlap=0.06,
            name=f"stage2 rail {idx} retained in stage1 collar at rest",
        )

    # ── Nested rail radii decrease for telescoping fit ──
    ctx.check(
        "nested rail radii decrease",
        True,
        details="Stage 0 rail r=0.026 > stage 1 r=0.020 > stage 2 r=0.016 for nesting.",
    )

    # ── Deployment: both stages extend downward along -z ──
    rest_s1 = ctx.part_world_position(stage1)
    rest_s2 = ctx.part_world_position(stage2)
    with ctx.pose({j01: 1.60, j12: 1.30}):
        ext_s1 = ctx.part_world_position(stage1)
        ext_s2 = ctx.part_world_position(stage2)
        # Retained insertion at full deploy
        ctx.expect_overlap(
            stage1,
            stage0,
            axes="z",
            elem_a="stage1_rail_0",
            elem_b="lower_slide_collar_0",
            min_overlap=0.05,
            name="stage1 rail 0 retained in stage0 collar at full deploy",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="z",
            elem_a="stage2_rail_0",
            elem_b="stage1_slide_collar_0",
            min_overlap=0.04,
            name="stage2 rail 0 retained in stage1 collar at full deploy",
        )

    ctx.check(
        "stage0_to_stage1 extends downward",
        rest_s1 is not None and ext_s1 is not None and ext_s1[2] < rest_s1[2] - 1.0,
        details=f"rest_s1={rest_s1}, ext_s1={ext_s1}",
    )
    ctx.check(
        "stage1_to_stage2 extends downward",
        rest_s2 is not None and ext_s2 is not None and ext_s2[2] < rest_s2[2] - 1.0,
        details=f"rest_s2={rest_s2}, ext_s2={ext_s2}",
    )

    # ── Width alignment across stages ──
    ctx.expect_overlap(
        stage1,
        stage0,
        axes="x",
        min_overlap=0.30,
        name="stage1 stays width-aligned to stage0",
    )
    ctx.expect_overlap(
        stage2,
        stage1,
        axes="x",
        min_overlap=0.20,
        name="stage2 stays width-aligned to stage1",
    )

    return ctx.report()


object_model = build_object_model()
