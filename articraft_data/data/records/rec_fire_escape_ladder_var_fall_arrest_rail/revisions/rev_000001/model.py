from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_fire_escape_ladder_fall_arrest",
        meta={
            "run_notes": (
                "Fork variant: replaced the cylindrical safety cage with a central vertical "
                "fall-arrest guide rail and travelling shuttle/clamp on the ladder centerline. "
                "Same wall-mounted fire escape ladder subcategory; modern code-compliant alternative "
                "to cages. Preserved side rails, indexed rungs, wall standoffs, and prismatic lower section."
            )
        },
    )

    galvanized = model.material("galvanized_steel", rgba=(0.72, 0.76, 0.75, 1.0))
    dark_steel = model.material("dark_bolt_heads", rgba=(0.08, 0.09, 0.09, 1.0))
    red_latch = model.material("red_release_latch", rgba=(0.9, 0.05, 0.03, 1.0))
    red_shuttle = model.material("red_shuttle_body", rgba=(0.85, 0.08, 0.06, 1.0))

    height = 5.25
    rail_x = 0.25
    rail_radius = 0.026
    rung_radius = 0.017

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

    # Wall stand-off brackets and mounting plates, without modeling the wall itself.
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

    # Central fall-arrest guide rail: a vertical tube on the ladder centerline,
    # mounted on short standoff brackets between the rungs.
    arrest_rail_bottom = 0.60
    arrest_rail_top = 5.10
    arrest_rail_length = arrest_rail_top - arrest_rail_bottom
    arrest_rail_radius = 0.016
    _cyl_z(
        upper,
        "arrest_rail",
        arrest_rail_length,
        arrest_rail_radius,
        (0.0, 0.0, (arrest_rail_bottom + arrest_rail_top) / 2.0),
        galvanized,
    )

    # Short standoff brackets tying the arrest rail to the ladder rungs.
    # Place them at midpoints between consecutive rung heights so they don't
    # collide with the rungs. Brackets are short horizontal arms from the
    # rung plane to the rail surface.
    bracket_z_levels = (0.93, 1.53, 2.13, 2.73, 3.33, 3.93, 4.53)
    for bracket_index, z in enumerate(bracket_z_levels):
        # Two short cross-bars connecting the rail to the side rails.
        _cyl_x(
            upper,
            f"arrest_bracket_{bracket_index}",
            rail_x - arrest_rail_radius + 0.01,
            0.012,
            (rail_x / 2.0, 0.0, z),
            galvanized,
        )
        _cyl_x(
            upper,
            f"arrest_bracket_mirror_{bracket_index}",
            rail_x - arrest_rail_radius + 0.01,
            0.012,
            (-rail_x / 2.0, 0.0, z),
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

    # Travelling shuttle/clamp that captures the fall-arrest rail.
    # The shuttle body wraps around the rail and has an attachment eye for
    # a lanyard. Modeled as its own part on a PRISMATIC joint along the rail.
    shuttle = model.part("arrest_shuttle")
    # Main shuttle body: a compact block that reads as a clamp sliding on the rail.
    shuttle.visual(
        Box((0.08, 0.06, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=red_shuttle,
        name="shuttle_body",
    )
    # Attachment eye: a small cylinder projecting sideways for lanyard clip.
    _cyl_y(shuttle, "shuttle_eye", 0.05, 0.008, (0.0, 0.055, 0.0), galvanized)

    # Prismatic joint: shuttle slides along the arrest rail (world Z axis).
    # Origin at a midpoint between rung levels to avoid rung collision at rest.
    # Rungs are at z = 0.18 + i*0.30; pick z = 2.43 (between rung 7 at 2.28 and rung 8 at 2.58).
    shuttle_origin_z = 2.43
    shuttle_half_travel = (arrest_rail_length / 2.0) - 0.10
    model.articulation(
        "shuttle_to_rail",
        ArticulationType.PRISMATIC,
        parent=upper,
        child=shuttle,
        origin=Origin(xyz=(0.0, 0.0, shuttle_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.5,
            lower=-shuttle_half_travel,
            upper=shuttle_half_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_ladder")
    lower = object_model.get_part("lower_ladder")
    shuttle = object_model.get_part("arrest_shuttle")
    slide = object_model.get_articulation("ladder_to_lower")
    shuttle_joint = object_model.get_articulation("shuttle_to_rail")

    # Lower ladder slide-collar capture (preserved from parent).
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

    # Shuttle captures the arrest rail (intentional nesting: shuttle wraps rail).
    ctx.allow_overlap(
        upper,
        shuttle,
        elem_a="arrest_rail",
        elem_b="shuttle_body",
        reason=(
            "The travelling shuttle clamp intentionally wraps around the central "
            "fall-arrest guide rail to represent a captured sliding fit."
        ),
    )
    ctx.expect_within(
        upper,
        shuttle,
        axes="xy",
        inner_elem="arrest_rail",
        outer_elem="shuttle_body",
        margin=0.005,
        name="arrest rail is captured within shuttle body in XY",
    )
    ctx.expect_overlap(
        shuttle,
        upper,
        axes="z",
        elem_a="shuttle_body",
        elem_b="arrest_rail",
        min_overlap=0.05,
        name="shuttle body overlaps the arrest rail along Z (captured fit)",
    )

    # Structural checks.
    ctx.check(
        "regular fixed rungs",
        len([v for v in upper.visuals if v.name.startswith("rung_")]) >= 16,
        details="The fixed ladder should show a full run of evenly spaced rungs.",
    )
    ctx.check(
        "fall arrest rail present",
        any(v.name == "arrest_rail" for v in upper.visuals),
        details="Central vertical fall-arrest guide rail must be present on the ladder centerline.",
    )
    ctx.check(
        "arrest rail brackets",
        len([v for v in upper.visuals if v.name.startswith("arrest_bracket_")]) >= 6,
        details="Multiple standoff brackets should tie the arrest rail to the ladder structure.",
    )
    ctx.check(
        "shuttle has eye",
        any(v.name == "shuttle_eye" for v in shuttle.visuals),
        details="Shuttle must have an attachment eye for lanyard connection.",
    )
    ctx.check(
        "wall mounting hardware",
        len([v for v in upper.visuals if v.name.startswith("wall_plate_")]) >= 10,
        details="Side stand-off wall plates are required to read as a wall-mounted ladder.",
    )

    # Lower extension alignment and guide plane.
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

    # Lower prismatic slide test.
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

    # Shuttle prismatic travel test.
    shuttle_rest = ctx.part_world_position(shuttle)
    shuttle_upper_limit = shuttle_joint.motion_limits.upper
    with ctx.pose({shuttle_joint: shuttle_upper_limit}):
        shuttle_high = ctx.part_world_position(shuttle)
    ctx.check(
        "shuttle slides upward along arrest rail",
        shuttle_rest is not None
        and shuttle_high is not None
        and shuttle_high[2] > shuttle_rest[2] + 1.0,
        details=f"shuttle rest={shuttle_rest}, shuttle high={shuttle_high}",
    )

    return ctx.report()


object_model = build_object_model()
