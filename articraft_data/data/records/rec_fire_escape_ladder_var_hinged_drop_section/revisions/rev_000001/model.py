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
        name="wall_fire_escape_ladder_cage",
        meta={
            "run_notes": (
                "Modeled as a wall-mounted fire escape ladder with cage and side mounting hardware. "
                "The supplied reference image is consistent with the requested category; no classification conflict suspected."
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

    # Hinge bracket ears at the bottom of the upper ladder that carry the swing pin.
    hinge_z = 0.06
    for side_index, x in enumerate((-rail_x, rail_x)):
        # Flat ear plate projecting forward from each rail.
        upper.visual(
            Box((0.070, 0.120, 0.090)),
            origin=Origin(xyz=(x, 0.060, hinge_z)),
            material=galvanized,
            name=f"hinge_ear_{side_index}",
        )
        # Gusset web tying the ear back into the rail.
        _cyl_y(upper, f"hinge_gusset_{side_index}", 0.10, 0.014, (x, 0.035, hinge_z), galvanized)
    # Cross brace tying the two hinge ears together at the bottom.
    _cyl_x(upper, "hinge_cross_brace", 2.0 * rail_x + 0.04, 0.016, (0.0, 0.060, hinge_z), galvanized)

    lower = model.part("lower_ladder")
    lower_rail_x = 0.18
    lower_rail_radius = 0.022
    # The lower section is drawn in its stowed (folded-up) configuration:
    # rails extend upward in local +Z from the hinge pivot at the part origin.
    # At q=0 the child frame aligns with the articulation frame (stowed).
    # Positive q swings the section outward and down around the X-axis hinge.
    lower_y = 0.06  # local Y offset — places lower rails slightly forward of upper rails when stowed
    lower_sleeve_top = 0.12
    lower_rail_start = lower_sleeve_top + 0.02
    lower_rail_end = 2.10
    lower_rail_length = lower_rail_end - lower_rail_start
    lower_rail_mid = (lower_rail_start + lower_rail_end) / 2.0

    # Swing hinge hardware: sleeve and pin captured between the upper hinge ears.
    _cyl_x(lower, "swing_sleeve", 2.0 * lower_rail_x + 0.06, 0.032, (0.0, 0.0, 0.0), galvanized)
    _cyl_x(lower, "swing_pin", 2.0 * lower_rail_x + 0.10, 0.018, (0.0, 0.0, 0.0), dark_steel)
    # Pin head and cotter at each end of the swing pin.
    for side_index, x_end in enumerate((-lower_rail_x - 0.05, lower_rail_x + 0.05)):
        _cyl_x(lower, f"swing_pin_cap_{side_index}", 0.018, 0.028, (x_end, 0.0, 0.0), dark_steel)

    # Hinge bracket arms connecting the sleeve/pin to the lower rails.
    # These run from the hinge pivot (z=0) up to the rail start, forming the
    # structural connection that ties the hinge hardware to the ladder frame.
    bracket_length = lower_rail_start
    bracket_mid_z = bracket_length / 2.0
    for side_index, x in enumerate((-lower_rail_x, lower_rail_x)):
        _cyl_z(lower, f"hinge_bracket_{side_index}", bracket_length + 0.04, 0.018, (x, lower_y / 2.0, bracket_mid_z), galvanized)

    # Lower ladder rails (extending upward in stowed local frame).
    _cyl_z(lower, "lower_rail_0", lower_rail_length, lower_rail_radius, (-lower_rail_x, lower_y, lower_rail_mid), galvanized)
    _cyl_z(lower, "lower_rail_1", lower_rail_length, lower_rail_radius, (lower_rail_x, lower_y, lower_rail_mid), galvanized)

    # Rungs along the lower section (in stowed local frame, climbing upward).
    rung_zs = [lower_rail_start + 0.15 + 0.30 * i for i in range(6)]
    for rung_index, z in enumerate(rung_zs):
        _cyl_x(lower, f"lower_rung_{rung_index}", 2.0 * lower_rail_x + 0.04, 0.016, (0.0, lower_y, z), galvanized)

    # Foot bar at the far end of the lower section (becomes the bottom when deployed).
    _cyl_x(lower, "lower_foot_bar", 2.0 * lower_rail_x + 0.12, 0.022, (0.0, lower_y, lower_rail_end), galvanized)

    # Counterweight arm: extends in local -Z from the hinge, providing counterbalance.
    # When stowed (q=0) this hangs down below the hinge; when deployed it swings up.
    counterweight_length = 0.45
    counterweight_radius = 0.040
    _cyl_z(lower, "counterweight_arm", counterweight_length, 0.016, (0.0, 0.0, -counterweight_length / 2.0), galvanized)
    _cyl_z(lower, "counterweight_mass", 0.12, counterweight_radius, (0.0, 0.0, -counterweight_length + 0.06), dark_steel)

    # Latch tab on the lower section for stowed retention.
    lower.visual(
        Box((0.12, 0.024, 0.045)),
        origin=Origin(xyz=(0.0, lower_y + 0.020, lower_sleeve_top + 0.06)),
        material=red_latch,
        name="lower_latch_tab",
    )

    model.articulation(
        "ladder_to_lower",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=lower,
        origin=Origin(xyz=(0.0, 0.06, 0.06)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=0.0, upper=2.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_ladder")
    lower = object_model.get_part("lower_ladder")
    swing = object_model.get_articulation("ladder_to_lower")

    # The swing sleeve and pin pass through the upper hinge ears and cross brace;
    # small intentional overlap at the hinge assembly is the real pivot capture.
    for side_index in (0, 1):
        ctx.allow_overlap(
            lower,
            upper,
            elem_a="swing_sleeve",
            elem_b=f"hinge_ear_{side_index}",
            reason=(
                "The swing sleeve is intentionally captured between the hinge bracket ears "
                "so the pivoting lower section remains mounted on the fixed ladder hinge."
            ),
        )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="swing_sleeve",
        elem_b="hinge_cross_brace",
        reason=(
            "The swing sleeve passes through the hinge cross brace connecting the two bracket "
            "ears; this is the real pivot axis of the counterweighted drop section."
        ),
    )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="swing_pin",
        elem_b="hinge_cross_brace",
        reason=(
            "The swing pin passes through the hinge cross brace; this is the real pivot axis "
            "of the counterweighted drop section hinge."
        ),
    )
    for side_index in (0, 1):
        ctx.allow_overlap(
            lower,
            upper,
            elem_a="swing_pin",
            elem_b=f"hinge_ear_{side_index}",
            reason=(
                "The swing pin passes through the hinge ear; this is the real pivot axis "
                "of the counterweighted drop section hinge."
            ),
        )
        ctx.allow_overlap(
            lower,
            upper,
            elem_a=f"swing_pin_cap_{side_index}",
            elem_b=f"hinge_ear_{side_index}",
            reason=(
                "The swing pin cap is intentionally captured outside the hinge ear to retain "
                "the pivot pin; this is the real hinge capture mechanism."
            ),
        )
        ctx.allow_overlap(
            lower,
            upper,
            elem_a=f"swing_pin_cap_{side_index}",
            elem_b="hinge_cross_brace",
            reason=(
                "The swing pin cap overlaps with the hinge cross brace; this is the real hinge "
                "capture mechanism that retains the pivot pin."
            ),
        )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="counterweight_arm",
        elem_b="hinge_cross_brace",
        reason=(
            "The counterweight arm passes by the hinge cross brace when the ladder swings down; "
            "this is the real counterbalance mechanism for the drop section."
        ),
    )

    # Structural checks preserved from the parent baseline.
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
    ctx.check(
        "wall mounting hardware",
        len([v for v in upper.visuals if v.name.startswith("wall_plate_")]) >= 10,
        details="Side stand-off wall plates are required to read as a wall-mounted ladder.",
    )

    # Variant-specific: the lower section must have a visible counterweight arm
    # and hinge pin/sleeve hardware proving the revolute swing-down mechanism.
    lower_visual_names = {v.name for v in lower.visuals}
    ctx.check(
        "counterweight arm present",
        "counterweight_arm" in lower_visual_names and "counterweight_mass" in lower_visual_names,
        details="The counterweighted drop section requires a visible counterweight arm and mass.",
    )
    ctx.check(
        "swing hinge hardware present",
        "swing_pin" in lower_visual_names and "swing_sleeve" in lower_visual_names,
        details="The revolute swing hinge requires a visible pin and sleeve on the lower section.",
    )
    ctx.check(
        "lower section rungs",
        len([v for v in lower.visuals if v.name.startswith("lower_rung_")]) >= 5,
        details="The drop-down ladder section must have its own climbing rungs.",
    )

    # The articulation must be REVOLUTE (not the parent prismatic slide).
    ctx.check(
        "ladder_to_lower is revolute",
        swing.articulation_type == ArticulationType.REVOLUTE,
        details=f"Expected REVOLUTE swing hinge but got {swing.articulation_type}.",
    )

    # Pose checks: stowed (q=0) has the lower section folded up; deployed (q=upper)
    # swings it down to extend the ladder toward the ground.
    # Check the position of lower_rung_5 element, not the part origin.
    with ctx.pose({swing: 0.0}):
        stowed_rung_pos = ctx.part_element_world_aabb(lower, elem="lower_rung_5")
    with ctx.pose({swing: 2.95}):
        deployed_rung_pos = ctx.part_element_world_aabb(lower, elem="lower_rung_5")

    if stowed_rung_pos is not None and deployed_rung_pos is not None:
        stowed_z_center = (stowed_rung_pos[0][2] + stowed_rung_pos[1][2]) / 2.0
        deployed_z_center = (deployed_rung_pos[0][2] + deployed_rung_pos[1][2]) / 2.0
        ctx.check(
            "lower section swings down when deployed",
            deployed_z_center < stowed_z_center - 0.80,
            details=f"stowed rung z={stowed_z_center:.3f}, deployed rung z={deployed_z_center:.3f}",
        )

    # At stowed pose the lower rails should be above the hinge origin (folded up).
    # At deployed pose they should be below the hinge (hanging down for escape).
    with ctx.pose({swing: 0.0}):
        stowed_rail_pos = ctx.part_element_world_aabb(lower, elem="lower_rail_0")
    with ctx.pose({swing: 2.95}):
        deployed_rail_pos = ctx.part_element_world_aabb(lower, elem="lower_rail_0")

    if stowed_rail_pos is not None and deployed_rail_pos is not None:
        stowed_z_center = (stowed_rail_pos[0][2] + stowed_rail_pos[1][2]) / 2.0
        deployed_z_center = (deployed_rail_pos[0][2] + deployed_rail_pos[1][2]) / 2.0
        ctx.check(
            "stowed lower rails are above hinge",
            stowed_z_center > 0.06,
            details=f"stowed rail z center={stowed_z_center:.3f}, hinge z=0.06",
        )
        ctx.check(
            "deployed lower rails drop below hinge",
            deployed_z_center < 0.06,
            details=f"deployed rail z center={deployed_z_center:.3f}, hinge z=0.06",
        )

    return ctx.report()


object_model = build_object_model()
