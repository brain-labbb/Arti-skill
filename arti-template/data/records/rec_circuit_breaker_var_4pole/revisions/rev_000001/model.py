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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="four_pole_din_rail_circuit_breaker",
        meta={
            "category": "Electrical_Wiring",
            "small_class": "Circuit breaker",
            "source": "picture/Electrical_Wiring/Circuit breaker/001.png",
        },
    )

    white = model.material("warm_white_molded_plastic", rgba=(0.92, 0.95, 0.95, 1.0))
    off_white = model.material("slightly_recessed_white", rgba=(0.82, 0.87, 0.88, 1.0))
    blue = model.material("blue_handle_plastic", rgba=(0.00, 0.28, 0.70, 1.0))
    cyan = model.material("cyan_brand_stripe", rgba=(0.00, 0.62, 0.78, 1.0))
    dark = model.material("dark_screw_cavity", rgba=(0.015, 0.017, 0.018, 1.0))
    screw = model.material("darkened_phillips_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    grey = model.material("grey_embossed_marking", rgba=(0.45, 0.48, 0.50, 1.0))
    black_print = model.material("black_printing", rgba=(0.02, 0.025, 0.025, 1.0))

    # ── Pole layout ──────────────────────────────────────────────────────
    # 4-pole DIN main breaker: 18 mm per pole, centred on the housing.
    pole_spacing = 0.018
    num_poles = 4
    pole_x = tuple(
        (i - (num_poles - 1) / 2.0) * pole_spacing for i in range(num_poles)
    )  # (-0.027, -0.009, 0.009, 0.027)

    housing_width = num_poles * pole_spacing  # 0.072

    # ── Housing (fixed root) ─────────────────────────────────────────────
    housing = model.part("housing")

    # Main molded case: 72 mm wide (4P), 86 mm tall, 72 mm deep.
    housing.visual(
        Box((housing_width, 0.072, 0.086)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white,
        name="molded_case",
    )

    # Stepped terminal shelves and the slightly raised front label area.
    housing.visual(
        Box((housing_width + 0.001, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.039, 0.036)),
        material=white,
        name="top_terminal_block",
    )
    housing.visual(
        Box((housing_width + 0.001, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.039, -0.038)),
        material=white,
        name="bottom_terminal_block",
    )
    housing.visual(
        Box((housing_width - 0.005, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, -0.0405, 0.005)),
        material=white,
        name="front_label_area",
    )

    # Fixed pole separation ribs (inner separators between adjacent poles).
    # 4 poles → 3 inner separators at the midpoints between adjacent poles.
    for i in range(num_poles - 1):
        sep_x = (pole_x[i] + pole_x[i + 1]) / 2.0
        housing.visual(
            Box((0.0030, 0.008, 0.053)),
            origin=Origin(xyz=(sep_x, -0.0430, 0.004)),
            material=white,
            name=f"fixed_white_separator_{i}",
        )

    # Outer pole ribs at the housing side walls.
    outer_edge_offset = pole_spacing * (num_poles - 1) / 2.0 + pole_spacing / 2.0 - 0.001
    for i, x in enumerate((-outer_edge_offset, outer_edge_offset)):
        housing.visual(
            Box((0.0022, 0.006, 0.060)),
            origin=Origin(xyz=(x, -0.0418, 0.001)),
            material=white,
            name=f"outer_pole_rib_{i}",
        )

    # Fixed white pivot frames around the blue rotating cylinder.
    # Inner pivot frames between adjacent poles.
    pivot_y = -0.0465
    pivot_z = -0.013
    for i in range(num_poles - 1):
        frame_x = (pole_x[i] + pole_x[i + 1]) / 2.0
        housing.visual(
            Box((0.0040, 0.0065, 0.022)),
            origin=Origin(xyz=(frame_x, pivot_y + 0.0018, pivot_z + 0.003)),
            material=white,
            name=f"fixed_white_pivot_frame_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0066, length=0.0018),
            origin=Origin(
                xyz=(frame_x, pivot_y - 0.0018, pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=off_white,
            name=f"fixed_white_pivot_socket_{i}",
        )

    # Outer pivot cheeks at the side walls.
    outer_cheek_x = outer_edge_offset + 0.0015
    for i, x in enumerate((-outer_cheek_x, outer_cheek_x)):
        housing.visual(
            Box((0.0030, 0.006, 0.017)),
            origin=Origin(xyz=(x, pivot_y + 0.0015, pivot_z)),
            material=white,
            name=f"fixed_white_outer_pivot_cheek_{i}",
        )

    # Dark pocket shadows at each pole position (visible recess for the drum).
    for i, x in enumerate(pole_x):
        housing.visual(
            Cylinder(radius=0.0072, length=0.0020),
            origin=Origin(
                xyz=(x, pivot_y + 0.0005, pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark,
            name=f"blue_cylinder_pocket_shadow_{i}",
        )

    # ── Terminal cavities, screws, wire ports (per pole, top + bottom) ───
    screw_z = (0.033, -0.037)
    for row, z in enumerate(screw_z):
        for col, x in enumerate(pole_x):
            housing.visual(
                Cylinder(radius=0.0044, length=0.004),
                origin=Origin(
                    xyz=(x, -0.0434, z), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material=dark,
                name=f"terminal_cavity_{row}_{col}",
            )
            housing.visual(
                Cylinder(radius=0.0029, length=0.0048),
                origin=Origin(
                    xyz=(x, -0.0450, z), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material=screw,
                name=f"terminal_screw_{row}_{col}",
            )
            housing.visual(
                Box((0.0056, 0.0007, 0.0009)),
                origin=Origin(
                    xyz=(x, -0.0477, z),
                    rpy=(0.0, 0.0, (-0.55 if col % 2 else 0.55)),
                ),
                material=grey,
                name=f"screw_slot_{row}_{col}",
            )

    # Top and bottom wire-entry ports.
    for row, z in enumerate((0.045, -0.046)):
        for col, x in enumerate(pole_x):
            housing.visual(
                Box((0.012, 0.018, 0.0016)),
                origin=Origin(xyz=(x, -0.039, z)),
                material=dark,
                name=f"wire_entry_port_{row}_{col}",
            )
            housing.visual(
                Box((0.010, 0.010, 0.0015)),
                origin=Origin(xyz=(x, -0.039, z)),
                material=off_white,
                name=f"recessed_terminal_lid_{row}_{col}",
            )

    # ── Front printed markings (per pole) ────────────────────────────────
    for col, x in enumerate(pole_x):
        housing.visual(
            Box((0.0135, 0.0014, 0.0038)),
            origin=Origin(xyz=(x, -0.0424, 0.023)),
            material=cyan,
            name=f"cyan_stripe_{col}",
        )
        housing.visual(
            Box((0.0105, 0.0008, 0.0026)),
            origin=Origin(xyz=(x, -0.0419, 0.0175)),
            material=cyan,
            name=f"cyan_rating_block_{col}",
        )
        for i, (w, z) in enumerate(
            (
                (0.0120, 0.0115),
                (0.0100, 0.0075),
                (0.0085, 0.0035),
                (0.0105, -0.0005),
                (0.0090, -0.0045),
            )
        ):
            housing.visual(
                Box((w, 0.0007, 0.0012)),
                origin=Origin(xyz=(x, -0.0419, z)),
                material=black_print,
                name=f"pole_rating_text_{col}_{i}",
            )
        # Per-pole contact indicator window.
        housing.visual(
            Box((0.0045, 0.0007, 0.0045)),
            origin=Origin(xyz=(x + 0.003, -0.0419, 0.002)),
            material=off_white,
            name=f"contact_indicator_bezel_{col}",
        )
        housing.visual(
            Box((0.0026, 0.0009, 0.0020)),
            origin=Origin(xyz=(x + 0.003, -0.0419, 0.002)),
            material=dark,
            name=f"green_indicator_window_{col}",
        )

    for col, x in enumerate(pole_x):
        for mark in range(3):
            housing.visual(
                Box((0.0011, 0.0006, 0.0040)),
                origin=Origin(
                    xyz=(x - 0.003 + 0.003 * mark, -0.0419, -0.0065)
                ),
                material=black_print,
                name=f"terminal_symbol_{col}_{mark}",
            )

    # ── Side ribs, vents, fasteners ──────────────────────────────────────
    side_x = outer_edge_offset
    for side, x in (("side_a", -side_x), ("side_b", side_x)):
        housing.visual(
            Box((0.0015, 0.045, 0.0018)),
            origin=Origin(xyz=(x, -0.003, 0.018)),
            material=off_white,
            name=f"{side}_horizontal_rib_top",
        )
        housing.visual(
            Box((0.0015, 0.040, 0.0018)),
            origin=Origin(xyz=(x, -0.005, -0.017)),
            material=off_white,
            name=f"{side}_horizontal_rib_bottom",
        )
        for n, z in enumerate((-0.006, -0.011, -0.016, -0.021)):
            housing.visual(
                Box((0.0017, 0.016, 0.0012)),
                origin=Origin(xyz=(x, 0.024, z)),
                material=grey,
                name=f"{side}_side_vent_{n}",
            )
        for n, z in enumerate((0.030, 0.000, -0.030)):
            housing.visual(
                Cylinder(radius=0.0022, length=0.0018),
                origin=Origin(
                    xyz=(x, -0.018, z), rpy=(0.0, math.pi / 2.0, 0.0)
                ),
                material=grey,
                name=f"{side}_case_fastener_{n}",
            )

    # DIN rail latch at the rear/lower side.
    housing.visual(
        Box((0.052, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0390, -0.024)),
        material=grey,
        name="fixed_din_rail_latch",
    )
    housing.visual(
        Box((0.046, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0375, -0.016)),
        material=dark,
        name="rear_rail_channel_shadow",
    )

    # ── Blue handle assembly (the ONLY moving part) ─────────────────────
    handle = model.part("blue_handle_assembly")

    # Shared pivot cylinder spanning all 4 poles.
    pivot_cyl_length = housing_width + 0.008  # slight overhang past outer cheeks
    handle.visual(
        Cylinder(radius=0.0034, length=pivot_cyl_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="blue_common_pivot_cylinder",
    )

    # Per-pole rotor drums, index ribs, toggle paddles, and OFF prints —
    # all emitted by ONE loop over the 4 pole positions.
    for col, x in enumerate(pole_x):
        # Rotor drum around the shared cylinder.
        handle.visual(
            Cylinder(radius=0.0062, length=0.0115),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blue,
            name=f"blue_rotor_drum_{col}",
        )
        # Index rib on each drum.
        handle.visual(
            Box((0.0095, 0.0018, 0.0018)),
            origin=Origin(xyz=(x, -0.0060, 0.0)),
            material=blue,
            name=f"blue_rotor_index_rib_{col}",
        )
        # Toggle paddle extending forward from the drum.
        handle.visual(
            Box((0.0105, 0.006, 0.021)),
            origin=Origin(xyz=(x, -0.0065, 0.0010)),
            material=blue,
            name=f"blue_toggle_paddle_{col}",
        )
        # OFF print on each paddle face.
        handle.visual(
            Box((0.0080, 0.0009, 0.0020)),
            origin=Origin(xyz=(x, -0.0098, 0.0070)),
            material=cyan,
            name=f"off_print_{col}",
        )

    # Lower horizontal tie bar connecting all 4 paddles.
    tie_bar_length = housing_width + 0.006
    handle.visual(
        Box((tie_bar_length, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.0092, -0.0135)),
        material=blue,
        name="blue_tie_bar",
    )
    handle.visual(
        Box((tie_bar_length - 0.006, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.0132, -0.0185)),
        material=blue,
        name="tie_bar_front_lip",
    )

    # ── Articulation ─────────────────────────────────────────────────────
    model.articulation(
        "housing_to_blue_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(0.0, -0.0465, -0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.42, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # ── Category check ───────────────────────────────────────────────────
    ctx.check(
        "small class is Circuit breaker",
        object_model.meta.get("small_class") == "Circuit breaker",
        details=str(object_model.meta),
    )

    # ── Joint structure ──────────────────────────────────────────────────
    hinge = object_model.get_articulation("housing_to_blue_handle")
    limits = hinge.motion_limits
    bounded_revolute = (
        hinge.child == "blue_handle_assembly"
        and hinge.parent == "housing"
        and str(hinge.articulation_type).lower().endswith("revolute")
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < limits.upper
    )
    ctx.check("one bounded blue handle revolute joint", bounded_revolute)

    revolute_children = [
        joint.child
        for joint in object_model.articulations
        if str(joint.articulation_type).lower().endswith("revolute")
    ]
    ctx.check(
        "only blue handle assembly is revolute child",
        revolute_children == ["blue_handle_assembly"],
        details=str(revolute_children),
    )

    # ── Fixed white geometry stays on housing ────────────────────────────
    housing = object_model.get_part("housing")
    handle = object_model.get_part("blue_handle_assembly")

    white_separator_names = sorted(
        v.name for v in housing.visuals if v.name.startswith("fixed_white_separator_")
    )
    expected_separators = [f"fixed_white_separator_{i}" for i in range(3)]
    ctx.check(
        "fixed white separators are housing geometry (3 separators for 4 poles)",
        white_separator_names == expected_separators,
        details=str(white_separator_names),
    )

    white_pivot_frame_names = sorted(
        v.name for v in housing.visuals if v.name.startswith("fixed_white_pivot_frame_")
    )
    expected_frames = [f"fixed_white_pivot_frame_{i}" for i in range(3)]
    ctx.check(
        "fixed white pivot frames receive the blue cylinder (3 frames for 4 poles)",
        white_pivot_frame_names == expected_frames,
        details=str(white_pivot_frame_names),
    )

    # ── Symmetric front markings across all 4 poles ──────────────────────
    housing_visual_names = {v.name for v in housing.visuals}
    symmetric_prints = (
        {f"cyan_stripe_{col}" for col in range(4)}
        | {f"cyan_rating_block_{col}" for col in range(4)}
        | {f"pole_rating_text_{col}_{line}" for col in range(4) for line in range(5)}
    )
    ctx.check(
        "front printed texture is repeated symmetrically on all four poles",
        symmetric_prints.issubset(housing_visual_names),
        details=f"missing={sorted(symmetric_prints - housing_visual_names)}",
    )

    # ── Moving assembly contains all 4 paddles, drums, and tie bar ──────
    moving_visual_names = {v.name for v in handle.visuals}
    expected_moving = {"blue_common_pivot_cylinder", "blue_tie_bar", "tie_bar_front_lip"}
    for col in range(4):
        expected_moving.update(
            {
                f"blue_rotor_drum_{col}",
                f"blue_toggle_paddle_{col}",
                f"blue_rotor_index_rib_{col}",
                f"off_print_{col}",
            }
        )
    ctx.check(
        "blue pivot cylinder, four paddles, four drums, and tie bar are one moving part",
        expected_moving.issubset(moving_visual_names),
        details=f"missing={sorted(expected_moving - moving_visual_names)}",
    )

    # ── No white geometry on the moving part ─────────────────────────────
    moving_material_names = {
        v.material.name if isinstance(v.material, Material) else str(v.material)
        for v in handle.visuals
    }
    no_white_moving = all(
        "white" not in mn.lower() for mn in moving_material_names
    ) and all("white" not in v.name.lower() for v in handle.visuals)
    ctx.check(
        "no white moving geometry in the blue handle assembly",
        no_white_moving,
        details=f"materials={moving_material_names}, visuals={sorted(moving_visual_names)}",
    )

    # ── Variant-specific: 4-pole multiplicity check ─────────────────────
    paddle_names = sorted(
        v.name for v in handle.visuals if v.name.startswith("blue_toggle_paddle_")
    )
    ctx.check(
        "four_pole_variant has exactly 4 blue toggle paddles emitted by loop",
        paddle_names == [f"blue_toggle_paddle_{i}" for i in range(4)],
        details=str(paddle_names),
    )

    drum_names = sorted(
        v.name for v in handle.visuals if v.name.startswith("blue_rotor_drum_")
    )
    ctx.check(
        "four_pole_variant has exactly 4 rotor drums on shared pivot cylinder",
        drum_names == [f"blue_rotor_drum_{i}" for i in range(4)],
        details=str(drum_names),
    )

    terminal_top = sorted(
        v.name for v in housing.visuals if v.name.startswith("terminal_screw_0_")
    )
    ctx.check(
        "four_pole_variant has 4 top terminal screw columns",
        terminal_top == [f"terminal_screw_0_{i}" for i in range(4)],
        details=str(terminal_top),
    )

    # ── Rest-pose spatial checks ─────────────────────────────────────────
    ctx.expect_gap(
        housing,
        handle,
        axis="y",
        min_gap=0.001,
        positive_elem="molded_case",
        name="blue handle assembly is proud of fixed white case",
    )
    ctx.expect_overlap(
        handle,
        housing,
        axes="x",
        min_overlap=0.060,
        elem_a="blue_tie_bar",
        elem_b="front_label_area",
        name="tie bar spans across all four poles",
    )
    ctx.expect_overlap(
        handle,
        housing,
        axes="x",
        min_overlap=0.003,
        elem_a="blue_common_pivot_cylinder",
        elem_b="fixed_white_pivot_frame_0",
        name="blue pivot cylinder is seated in fixed white frame",
    )
    ctx.expect_gap(
        housing,
        handle,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="fixed_white_separator_0",
        negative_elem="blue_toggle_paddle_0",
        name="fixed separator visually divides adjacent paddles",
    )

    # ── Articulated pose: only blue moves, housing stays fixed ──────────
    base_position = ctx.part_world_position(housing)
    paddle_rest = ctx.part_element_world_aabb(handle, elem="blue_toggle_paddle_1")
    bar_rest = ctx.part_element_world_aabb(handle, elem="blue_tie_bar")
    drum_rest = ctx.part_element_world_aabb(handle, elem="blue_rotor_drum_1")

    with ctx.pose({hinge: limits.upper if limits is not None else 0.3}):
        housing_position = ctx.part_world_position(housing)
        paddle_moved = ctx.part_element_world_aabb(handle, elem="blue_toggle_paddle_1")
        bar_moved = ctx.part_element_world_aabb(handle, elem="blue_tie_bar")
        drum_moved = ctx.part_element_world_aabb(handle, elem="blue_rotor_drum_1")

        ctx.check(
            "white housing remains fixed while handle rotates",
            base_position == housing_position,
            details=f"rest={base_position}, posed={housing_position}",
        )
        ctx.check(
            "blue paddle moves with revolute pose",
            paddle_rest is not None
            and paddle_moved is not None
            and abs(paddle_moved[0][2] - paddle_rest[0][2]) > 0.002,
            details=f"rest={paddle_rest}, moved={paddle_moved}",
        )
        ctx.check(
            "blue tie bar moves with same assembly",
            bar_rest is not None
            and bar_moved is not None
            and abs(bar_moved[0][2] - bar_rest[0][2]) > 0.002,
            details=f"rest={bar_rest}, moved={bar_moved}",
        )
        ctx.check(
            "blue rotor drum stays on pivot while rotating with the assembly",
            drum_rest is not None
            and drum_moved is not None
            and abs(drum_moved[0][1] - drum_rest[0][1]) < 0.002
            and abs(drum_moved[0][2] - drum_rest[0][2]) < 0.002,
            details=f"rest={drum_rest}, moved={drum_moved}",
        )

    return ctx.report()


object_model = build_object_model()
