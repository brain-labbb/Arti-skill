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
        name="sill_hook_fire_escape_ladder",
        meta={
            "run_notes": (
                "Portable rigid-rail fire escape ladder with semicircular safety cage, "
                "converted from wall-bolted mounting to an over-the-windowsill hook frame. "
                "The hook head folds via a revolute pivot for storage. "
                "Category: Emergency Equipment / Fire escape ladder; no classification conflict."
            )
        },
    )

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.76, 0.75, 1.0))
    dark_steel = model.material("dark_bolt_heads", rgba=(0.08, 0.09, 0.09, 1.0))
    red_latch = model.material("red_release_latch", rgba=(0.9, 0.05, 0.03, 1.0))

    height = 5.25
    rail_x = 0.25
    rail_radius = 0.026
    rung_radius = 0.017
    cage_radius = 0.58
    cage_tube = 0.017
    cage_bottom = 1.55
    cage_top = 5.12

    upper = model.part("upper_ladder")

    # Main fixed ladder rails.
    _cyl_z(upper, "rail_0", height, rail_radius, (-rail_x, 0.0, height / 2.0), galvanized)
    _cyl_z(upper, "rail_1", height, rail_radius, (rail_x, 0.0, height / 2.0), galvanized)

    # Regular climbing rungs, welded through the side rails.
    rung_z = 0.18
    rung_index = 0
    while rung_z < height - 0.10:
        _cyl_x(upper, f"rung_{rung_index}", 2.0 * rail_x + 0.045, rung_radius, (0.0, 0.0, rung_z), galvanized)
        rung_z += 0.30
        rung_index += 1

    # Short sill stand-off stubs on upper rungs keep the rails off the building wall.
    for side_index, x in enumerate((-rail_x, rail_x)):
        _cyl_y(upper, f"sill_standoff_{side_index}", 0.14, 0.018, (x, -0.07, height - 0.30), galvanized)

    # Cage hoops: semicircular horizontal guards projecting away from the wall side.
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
        # Short straight arms tie each hoop back into the ladder rail.
        left_mid = (-(rail_x + cage_radius) / 2.0, 0.0, z)
        right_mid = ((rail_x + cage_radius) / 2.0, 0.0, z)
        _cyl_x(upper, f"cage_arm_{hoop_index}_0", cage_radius - rail_x + 0.035, cage_tube, left_mid, galvanized)
        _cyl_x(upper, f"cage_arm_{hoop_index}_1", cage_radius - rail_x + 0.035, cage_tube, right_mid, galvanized)

    # Vertical cage bars following the semicircular envelope.
    cage_bar_height = cage_top - cage_bottom
    for bar_index, angle in enumerate((pi, 5 * pi / 6, 2 * pi / 3, pi / 2, pi / 3, pi / 6, 0.0)):
        x = cage_radius * cos(angle)
        y = cage_radius * sin(angle)
        _cyl_z(
            upper,
            f"cage_bar_{bar_index}",
            cage_bar_height + 0.06,
            0.014,
            (x, y, (cage_bottom + cage_top) / 2.0),
            galvanized,
        )

    # Lower extension guide hardware and visible release stops.
    for side_index, x in enumerate((-0.31, 0.31)):
        rail_side = -1.0 if x < 0.0 else 1.0
        _cyl_y(upper, f"lower_guide_cross_{side_index}", 0.14, 0.015, (x, 0.065, 0.22), galvanized)
        upper.visual(
            Box((0.11, 0.018, 0.28)),
            origin=Origin(xyz=(x, 0.155, 0.22)),
            material=galvanized,
            name=f"lower_guide_plate_{side_index}",
        )
        upper.visual(
            Box((0.10, 0.17, 0.060)),
            origin=Origin(xyz=(rail_side * 0.285, 0.075, 0.22)),
            material=galvanized,
            name=f"lower_guide_web_{side_index}",
        )
    for side_index, x in enumerate((-0.18, 0.18)):
        upper.visual(
            Box((0.080, 0.070, 0.100)),
            origin=Origin(xyz=(x, 0.105, 0.140)),
            material=galvanized,
            name=f"lower_slide_collar_{side_index}",
        )
        upper.visual(
            Box((0.075, 0.080, 0.070)),
            origin=Origin(xyz=(-0.250 if x < 0.0 else 0.250, 0.045, 0.140)),
            material=galvanized,
            name=f"lower_slide_bridge_{side_index}",
        )
    upper.visual(
        Box((0.18, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, 0.020, 0.460)),
        material=red_latch,
        name="release_latch",
    )

    lower = model.part("lower_ladder")
    lower_rail_x = 0.18
    lower_y = 0.10
    lower_top = 1.25
    lower_bottom = -1.12
    lower_height = lower_top - lower_bottom
    _cyl_z(lower, "lower_rail_0", lower_height, 0.022, (-lower_rail_x, lower_y, (lower_top + lower_bottom) / 2.0), galvanized)
    _cyl_z(lower, "lower_rail_1", lower_height, 0.022, (lower_rail_x, lower_y, (lower_top + lower_bottom) / 2.0), galvanized)
    for rung_index, z in enumerate((-0.95, -0.65, -0.35, -0.05)):
        _cyl_x(lower, f"lower_rung_{rung_index}", 2.0 * lower_rail_x + 0.04, 0.016, (0.0, lower_y, z), galvanized)
    _cyl_x(lower, "lower_foot_bar", 2.0 * lower_rail_x + 0.12, 0.022, (0.0, lower_y, lower_bottom), galvanized)
    lower.visual(
        Box((0.42, 0.024, 0.045)),
        origin=Origin(xyz=(0.0, lower_y + 0.020, 0.07)),
        material=red_latch,
        name="lower_latch_tab",
    )

    model.articulation(
        "ladder_to_lower",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.95),
    )

    # Sill hook assembly: a folding hook frame that hangs over a windowsill.
    hook = model.part("sill_hook")
    hook_pivot_z = height - 0.05
    hook_arm_length = 2.0 * rail_x + 0.12
    # Horizontal cross-arm spanning the ladder width at the pivot.
    _cyl_x(hook, "sill_hook_arm", hook_arm_length, 0.022, (0.0, 0.0, 0.0), galvanized)
    # Pivot hinge barrels at each rail junction.
    for side_index, x in enumerate((-rail_x, rail_x)):
        _cyl_x(hook, f"sill_hook_barrel_{side_index}", 0.06, 0.026, (x, 0.0, 0.0), dark_steel)
    # Two down-turned hook fingers extending toward the wall/building side (-Y).
    hook_throat = 0.28
    hook_drop = 0.22
    for side_index, x in enumerate((-rail_x + 0.06, rail_x - 0.06)):
        # Horizontal throat segment going toward the building wall.
        _cyl_y(hook, f"sill_hook_throat_{side_index}", hook_throat, 0.018, (x, -hook_throat / 2.0, 0.0), galvanized)
        # Down-turned finger at the end of the throat.
        _cyl_z(hook, f"sill_hook_finger_{side_index}", hook_drop, 0.018, (x, -hook_throat, -hook_drop / 2.0), galvanized)
        # Rounded tip cap on each finger.
        hook.visual(
            Box((0.04, 0.04, 0.016)),
            origin=Origin(xyz=(x, -hook_throat, -hook_drop - 0.008)),
            material=galvanized,
            name=f"sill_hook_tip_{side_index}",
        )

    model.articulation(
        "sill_hook_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=hook,
        # Pivot at the top of the ladder, on the rail plane.
        origin=Origin(xyz=(0.0, 0.0, hook_pivot_z)),
        # Rotation around X: positive q folds the hook fingers upward (stowed).
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_ladder")
    lower = object_model.get_part("lower_ladder")
    slide = object_model.get_articulation("ladder_to_lower")

    for index in (0, 1):
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"lower_slide_collar_{index}",
            elem_b=f"lower_rail_{index}",
            reason=(
                "The sliding lower ladder rail is intentionally captured inside a simplified "
                "guide collar so the movable extension remains mounted to the fixed ladder."
            ),
        )
        ctx.expect_within(
            lower,
            upper,
            axes="xy",
            inner_elem=f"lower_rail_{index}",
            outer_elem=f"lower_slide_collar_{index}",
            margin=0.0,
            name=f"lower rail {index} is centered in guide collar",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a=f"lower_rail_{index}",
            elem_b=f"lower_slide_collar_{index}",
            min_overlap=0.08,
            name=f"lower rail {index} remains inserted in guide collar",
        )

    ctx.check(
        "regular fixed rungs",
        len([v for v in upper.visuals if v.name.startswith("rung_")]) >= 16,
        details="The fixed ladder should show a full run of evenly spaced rungs.",
    )
    ctx.check(
        "semicircular cage hoops",
        len([v for v in upper.visuals if v.name.startswith("cage_hoop_")]) >= 6,
        details="The safety cage should be built from multiple horizontal semicircular hoops.",
    )
    hook = object_model.get_part("sill_hook")
    hook_pivot = object_model.get_articulation("sill_hook_pivot")

    ctx.check(
        "sill hook assembly",
        any(v.name == "sill_hook_arm" for v in hook.visuals)
        and sum(1 for v in hook.visuals if v.name.startswith("sill_hook_finger_")) >= 2,
        details="The hook frame must include a cross-arm and two down-turned hook fingers.",
    )
    ctx.check(
        "sill hook pivot is revolute",
        hook_pivot.articulation_type == ArticulationType.REVOLUTE,
        details="The sill hook must attach via a real REVOLUTE pivot for folding storage.",
    )

    # Sill standoffs on upper ladder
    ctx.check(
        "sill standoffs present",
        sum(1 for v in upper.visuals if v.name.startswith("sill_standoff_")) >= 2,
        details="Upper rungs should carry short standoff stubs to keep the rails off the wall.",
    )

    # Hinge barrels and cross-arm are intentionally captured around the ladder rails at the pivot.
    for side_index in (0, 1):
        ctx.allow_overlap(
            hook,
            upper,
            elem_a=f"sill_hook_barrel_{side_index}",
            elem_b=f"rail_{side_index}",
            reason=(
                "The hinge barrel is intentionally wrapped around the ladder rail "
                "to form the sill hook pivot hinge."
            ),
        )
        ctx.expect_overlap(
            hook,
            upper,
            axes="x",
            elem_a=f"sill_hook_barrel_{side_index}",
            elem_b=f"rail_{side_index}",
            min_overlap=0.01,
            name=f"sill hook barrel {side_index} engages rail at pivot",
        )

    # Cross-arm passes through the rails at the pivot hinge.
    ctx.allow_overlap(
        hook,
        upper,
        elem_a="sill_hook_arm",
        elem_b="rail_0",
        reason="The hook cross-arm is the hinge pin that passes through the rail at the pivot.",
    )
    ctx.allow_overlap(
        hook,
        upper,
        elem_a="sill_hook_arm",
        elem_b="rail_1",
        reason="The hook cross-arm is the hinge pin that passes through the rail at the pivot.",
    )
    ctx.expect_overlap(
        hook,
        upper,
        axes="x",
        elem_a="sill_hook_arm",
        elem_b="rail_0",
        min_overlap=0.02,
        name="sill hook arm spans across left rail at pivot",
    )

    # Hook pivot folding proof: check the hook finger element moves in Z when folded.
    rest_aabb = ctx.part_element_world_aabb(hook, elem="sill_hook_finger_0")
    with ctx.pose({hook_pivot: 1.2}):
        folded_aabb = ctx.part_element_world_aabb(hook, elem="sill_hook_finger_0")
    rest_z = rest_aabb[1][2] if rest_aabb else None
    folded_z = folded_aabb[1][2] if folded_aabb else None
    ctx.check(
        "sill hook folds for storage",
        rest_z is not None and folded_z is not None
        and abs(folded_z - rest_z) > 0.05,
        details=f"rest_z_max={rest_z}, folded_z_max={folded_z}",
    )
    ctx.expect_overlap(
        lower,
        upper,
        axes="x",
        min_overlap=0.35,
        name="lower extension stays aligned to ladder width",
    )
    ctx.expect_gap(
        lower,
        upper,
        axis="y",
        min_gap=0.010,
        max_gap=0.16,
        positive_elem="lower_rail_0",
        negative_elem="rail_0",
        name="sliding lower rails sit on front guide plane",
    )

    rest_pos = ctx.part_world_position(lower)
    with ctx.pose({slide: 0.95}):
        extended_pos = ctx.part_world_position(lower)
        ctx.expect_overlap(
            lower,
            upper,
            axes="x",
            min_overlap=0.35,
            name="extended lower section remains width aligned",
        )
    ctx.check(
        "lower section slides downward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.85,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
