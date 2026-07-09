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
        name="three_pole_din_rail_circuit_breaker",
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

    housing = model.part("housing")

    # Main molded case, approximately a real 3-pole MCB: 54 mm wide,
    # 86 mm tall, and 72 mm deep.
    housing.visual(
        Box((0.054, 0.072, 0.086)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=white,
        name="molded_case",
    )

    # Stepped terminal shelves and the slightly raised front label area.
    housing.visual(
        Box((0.055, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.039, 0.036)),
        material=white,
        name="top_terminal_block",
    )
    housing.visual(
        Box((0.055, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.039, -0.038)),
        material=white,
        name="bottom_terminal_block",
    )
    housing.visual(
        Box((0.049, 0.003, 0.038)),
        origin=Origin(xyz=(0.0, -0.0405, 0.005)),
        material=white,
        name="front_label_area",
    )

    # Fixed pole separation ribs. These are deliberately part of the housing:
    # they visually divide the blue paddles but never move with them.
    housing.visual(
        Box((0.0030, 0.008, 0.053)),
        origin=Origin(xyz=(-0.009, -0.0430, 0.004)),
        material=white,
        name="fixed_white_separator_0",
    )
    housing.visual(
        Box((0.0030, 0.008, 0.053)),
        origin=Origin(xyz=(0.009, -0.0430, 0.004)),
        material=white,
        name="fixed_white_separator_1",
    )
    for i, x in enumerate((-0.0275, 0.0275)):
        housing.visual(
            Box((0.0022, 0.006, 0.060)),
            origin=Origin(xyz=(x, -0.0418, 0.001)),
            material=white,
            name=f"outer_pole_rib_{i}",
        )

    # Fixed white pivot frames around the blue rotating cylinder.  These pieces
    # act like molded slots/bearing cheeks: the blue segmented cylinder passes
    # through them, but the white frames stay with the housing.
    pivot_y = -0.0465
    pivot_z = -0.013
    for i, x in enumerate((-0.009, 0.009)):
        housing.visual(
            Box((0.0040, 0.0065, 0.022)),
            origin=Origin(xyz=(x, pivot_y + 0.0018, pivot_z + 0.003)),
            material=white,
            name=f"fixed_white_pivot_frame_{i}",
        )
        housing.visual(
            Cylinder(radius=0.0066, length=0.0018),
            origin=Origin(xyz=(x, pivot_y - 0.0018, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=off_white,
            name=f"fixed_white_pivot_socket_{i}",
        )
    for i, x in enumerate((-0.029, 0.029)):
        housing.visual(
            Box((0.0030, 0.006, 0.017)),
            origin=Origin(xyz=(x, pivot_y + 0.0015, pivot_z)),
            material=white,
            name=f"fixed_white_outer_pivot_cheek_{i}",
        )
    for i, x in enumerate((-0.018, 0.0, 0.018)):
        housing.visual(
            Cylinder(radius=0.0072, length=0.0020),
            origin=Origin(xyz=(x, pivot_y + 0.0005, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"blue_cylinder_pocket_shadow_{i}",
        )

    # Round screw/terminal cavities on the front, with metal screw heads and slots.
    pole_x = (-0.018, 0.0, 0.018)
    screw_z = (0.033, -0.037)
    for row, z in enumerate(screw_z):
        for col, x in enumerate(pole_x):
            housing.visual(
                Cylinder(radius=0.0044, length=0.004),
                origin=Origin(xyz=(x, -0.0434, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark,
                name=f"terminal_cavity_{row}_{col}",
            )
            housing.visual(
                Cylinder(radius=0.0029, length=0.0048),
                origin=Origin(xyz=(x, -0.0450, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=screw,
                name=f"terminal_screw_{row}_{col}",
            )
            housing.visual(
                Box((0.0056, 0.0007, 0.0009)),
                origin=Origin(xyz=(x, -0.0477, z), rpy=(0.0, 0.0, (-0.55 if col % 2 else 0.55))),
                material=grey,
                name=f"screw_slot_{row}_{col}",
            )

    # Top and bottom wire-entry ports visible as dark rectangular slots.
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

    # Repeated printed marks on the fixed front face.  Each pole gets the same
    # pattern so the background texture reads symmetrical behind the handles.
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
                origin=Origin(xyz=(x - 0.003 + 0.003 * mark, -0.0419, -0.0065)),
                material=black_print,
                name=f"terminal_symbol_{col}_{mark}",
            )

    # Side molded embossing, vents, and fixed DIN rail latch at the rear/lower side.
    for side, x in (("side_a", -0.0275), ("side_b", 0.0275)):
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
                origin=Origin(xyz=(x, -0.018, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=grey,
                name=f"{side}_case_fastener_{n}",
            )

    housing.visual(
        Box((0.040, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0390, -0.024)),
        material=grey,
        name="fixed_din_rail_latch",
    )
    housing.visual(
        Box((0.034, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0375, -0.016)),
        material=dark,
        name="rear_rail_channel_shadow",
    )

    # The only moving part: one blue segmented cylinder seated in the fixed
    # white frames, three blue front paddles, and the lower blue tie bar.
    handle = model.part("blue_handle_assembly")
    handle.visual(
        Cylinder(radius=0.0034, length=0.061),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="blue_common_pivot_cylinder",
    )
    for i, x in enumerate(pole_x):
        handle.visual(
            Cylinder(radius=0.0062, length=0.0115),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blue,
            name=f"blue_rotor_drum_{i}",
        )
        handle.visual(
            Box((0.0095, 0.0018, 0.0018)),
            origin=Origin(xyz=(x, -0.0060, 0.0)),
            material=blue,
            name=f"blue_rotor_index_rib_{i}",
        )
    handle.visual(
        Box((0.0105, 0.006, 0.021)),
        origin=Origin(xyz=(-0.018, -0.0065, 0.0010)),
        material=blue,
        name="blue_toggle_paddle_0",
    )
    handle.visual(
        Box((0.0080, 0.0009, 0.0020)),
        origin=Origin(xyz=(-0.018, -0.0098, 0.0070)),
        material=cyan,
        name="off_print_0",
    )
    handle.visual(
        Box((0.0105, 0.006, 0.021)),
        origin=Origin(xyz=(0.0, -0.0065, 0.0010)),
        material=blue,
        name="blue_toggle_paddle_1",
    )
    handle.visual(
        Box((0.0080, 0.0009, 0.0020)),
        origin=Origin(xyz=(0.0, -0.0098, 0.0070)),
        material=cyan,
        name="off_print_1",
    )
    handle.visual(
        Box((0.0105, 0.006, 0.021)),
        origin=Origin(xyz=(0.018, -0.0065, 0.0010)),
        material=blue,
        name="blue_toggle_paddle_2",
    )
    handle.visual(
        Box((0.0080, 0.0009, 0.0020)),
        origin=Origin(xyz=(0.018, -0.0098, 0.0070)),
        material=cyan,
        name="off_print_2",
    )
    handle.visual(
        Box((0.060, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.0092, -0.0135)),
        material=blue,
        name="blue_tie_bar",
    )
    handle.visual(
        Box((0.054, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.0132, -0.0185)),
        material=blue,
        name="tie_bar_front_lip",
    )

    model.articulation(
        "housing_to_blue_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        # Horizontal front pivot line shared by all three paddles and the tie bar.
        origin=Origin(xyz=(0.0, -0.0465, -0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.42, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check(
        "small class is Circuit breaker",
        object_model.meta.get("small_class") == "Circuit breaker",
        details=str(object_model.meta),
    )

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

    housing = object_model.get_part("housing")
    handle = object_model.get_part("blue_handle_assembly")
    white_separator_names = [
        visual.name for visual in housing.visuals if visual.name.startswith("fixed_white_separator")
    ]
    white_pivot_frame_names = [
        visual.name for visual in housing.visuals if visual.name.startswith("fixed_white_pivot_frame")
    ]
    ctx.check(
        "fixed white separators are housing geometry",
        white_separator_names == ["fixed_white_separator_0", "fixed_white_separator_1"],
        details=str(white_separator_names),
    )
    ctx.check(
        "fixed white pivot frames receive the blue cylinder",
        white_pivot_frame_names == ["fixed_white_pivot_frame_0", "fixed_white_pivot_frame_1"],
        details=str(white_pivot_frame_names),
    )
    housing_visual_names = {visual.name for visual in housing.visuals}
    symmetric_prints = {
        f"cyan_stripe_{col}" for col in range(3)
    } | {
        f"cyan_rating_block_{col}" for col in range(3)
    } | {
        f"pole_rating_text_{col}_{line}" for col in range(3) for line in range(5)
    }
    ctx.check(
        "front printed texture is repeated symmetrically on all three poles",
        symmetric_prints.issubset(housing_visual_names),
        details=f"missing={sorted(symmetric_prints - housing_visual_names)}",
    )

    moving_visual_names = {visual.name for visual in handle.visuals}
    ctx.check(
        "blue segmented cylinder, three paddles, and tie bar are one moving part",
        {
            "blue_common_pivot_cylinder",
            "blue_rotor_drum_0",
            "blue_rotor_drum_1",
            "blue_rotor_drum_2",
            "blue_toggle_paddle_0",
            "blue_toggle_paddle_1",
            "blue_toggle_paddle_2",
            "blue_tie_bar",
        }.issubset(moving_visual_names),
        details=str(sorted(moving_visual_names)),
    )

    moving_material_names = {
        visual.material.name if isinstance(visual.material, Material) else str(visual.material)
        for visual in handle.visuals
    }
    no_white_moving_geometry = all(
        "white" not in material_name.lower() for material_name in moving_material_names
    ) and all(
        "white" not in visual.name.lower()
        for visual in handle.visuals
    )
    ctx.check(
        "no white moving geometry in the blue handle assembly",
        no_white_moving_geometry,
        details=f"materials={moving_material_names}, visuals={sorted(moving_visual_names)}",
    )

    # Rest pose: handle is proud of the fixed front case and divided by fixed
    # white separators in projection, without rotating any housing geometry.
    ctx.expect_gap(
        housing,
        handle,
        axis="y",
        min_gap=0.001,
        positive_elem="molded_case",
        negative_elem="blue_tie_bar",
        name="blue tie bar sits in front of fixed white case",
    )
    ctx.expect_overlap(
        handle,
        housing,
        axes="x",
        min_overlap=0.045,
        elem_a="blue_tie_bar",
        elem_b="front_label_area",
        name="tie bar spans across the three poles",
    )
    ctx.expect_overlap(
        handle,
        housing,
        axes="x",
        min_overlap=0.002,
        elem_a="blue_rotor_drum_1",
        elem_b="fixed_white_pivot_frame_0",
        name="blue segmented cylinder is seated in fixed white frame",
    )
    ctx.expect_gap(
        housing,
        handle,
        axis="x",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="fixed_white_separator_0",
        negative_elem="blue_toggle_paddle_0",
        name="fixed separator visually divides paddles",
    )

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
