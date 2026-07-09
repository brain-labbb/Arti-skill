from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _annular_z_sleeve(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length / 2.0, both=True)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length, both=True)
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_hydraulic_press")

    painted_gray = Material("painted_gray_steel", color=(0.55, 0.60, 0.62, 1.0))
    dark_gray = Material("dark_gray_steel", color=(0.11, 0.12, 0.13, 1.0))
    blue = Material("bright_blue_paint", color=(0.00, 0.45, 0.86, 1.0))
    black = Material("black_rubber", color=(0.005, 0.005, 0.006, 1.0))
    yellow = Material("hazard_yellow", color=(1.0, 0.75, 0.04, 1.0))
    red = Material("red_button", color=(0.85, 0.03, 0.02, 1.0))
    orange = Material("orange_button", color=(1.0, 0.36, 0.04, 1.0))
    green = Material("green_button", color=(0.0, 0.55, 0.12, 1.0))
    white = Material("white_button", color=(0.92, 0.92, 0.86, 1.0))
    silver = Material("bare_metal", color=(0.72, 0.73, 0.72, 1.0))
    plate_black = Material("black_plate", color=(0.02, 0.02, 0.018, 1.0))

    model.materials.extend(
        [
            painted_gray,
            dark_gray,
            blue,
            black,
            yellow,
            red,
            orange,
            green,
            white,
            silver,
            plate_black,
        ]
    )

    frame = model.part("frame")

    # One welded painted-steel scaffold: base cabinet, side uprights, and
    # massive crossbeams form a continuous workshop H-frame press.
    frame.visual(
        Box((1.12, 0.44, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=painted_gray,
        name="floor_foot",
    )
    frame.visual(
        Box((0.92, 0.34, 0.62)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=painted_gray,
        name="base_cabinet",
    )
    frame.visual(
        Box((0.10, 0.38, 1.75)),
        origin=Origin(xyz=(-0.51, 0.0, 0.96)),
        material=painted_gray,
        name="side_upright_0",
    )
    frame.visual(
        Box((0.10, 0.38, 1.75)),
        origin=Origin(xyz=(0.51, 0.0, 0.96)),
        material=painted_gray,
        name="side_upright_1",
    )
    frame.visual(
        Box((1.10, 0.32, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=painted_gray,
        name="lower_crossbeam",
    )
    frame.visual(
        Box((1.10, 0.30, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=painted_gray,
        name="top_crossbeam",
    )

    # Vertical guide columns/rods visible between the crossbeams.
    for x in (-0.43, 0.43):
        frame.visual(
            Cylinder(0.032, 1.13),
            origin=Origin(xyz=(x, -0.09, 1.285)),
            material=dark_gray,
            name=f"guide_column_{0 if x < 0 else 1}",
        )
        frame.visual(
            Cylinder(0.042, 0.08),
            origin=Origin(xyz=(x, -0.09, 0.86)),
            material=dark_gray,
            name=f"guide_column_collar_{0 if x < 0 else 1}",
        )
        frame.visual(
            Cylinder(0.042, 0.08),
            origin=Origin(xyz=(x, -0.09, 1.70)),
            material=dark_gray,
            name=f"guide_column_top_collar_{0 if x < 0 else 1}",
        )

    # Fixed blue hydraulic cylinder and gland sitting above and through the top beam.
    frame.visual(
        Cylinder(0.105, 0.52),
        origin=Origin(xyz=(0.0, -0.01, 2.19)),
        material=blue,
        name="hydraulic_cylinder",
    )
    frame.visual(
        Cylinder(0.122, 0.055),
        origin=Origin(xyz=(0.0, -0.01, 1.91)),
        material=blue,
        name="cylinder_cap",
    )
    frame.visual(
        mesh_from_cadquery(_annular_z_sleeve(0.070, 0.047, 0.12), "hollow_ram_gland"),
        origin=Origin(xyz=(0.0, -0.01, 1.76)),
        material=blue,
        name="ram_gland",
    )

    # Side-mounted blue control box and steel brackets tied into the right upright.
    frame.visual(
        Box((0.18, 0.16, 0.62)),
        origin=Origin(xyz=(0.72, -0.02, 1.18)),
        material=blue,
        name="control_box",
    )
    for z in (0.91, 1.45):
        frame.visual(
            Box((0.22, 0.055, 0.045)),
            origin=Origin(xyz=(0.61, -0.02, z)),
            material=dark_gray,
            name=f"control_bracket_{0 if z < 1.0 else 1}",
        )
    frame.visual(
        Box((0.105, 0.012, 0.58)),
        origin=Origin(xyz=(0.72, -0.106, 1.24)),
        material=white,
        name="button_panel",
    )

    # Small non-readable warning and data plates, modeled as colored plates only.
    frame.visual(
        Box((0.18, 0.010, 0.055)),
        origin=Origin(xyz=(0.02, -0.155, 1.83)),
        material=plate_black,
        name="top_warning_plate",
    )
    frame.visual(
        Box((0.028, 0.012, 0.036)),
        origin=Origin(xyz=(-0.07, -0.162, 1.835)),
        material=red,
        name="warning_red_block",
    )
    frame.visual(
        Box((0.055, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.172, 0.58)),
        material=silver,
        name="base_warning_plate",
    )
    frame.visual(
        Sphere(0.014),
        origin=Origin(xyz=(0.0, -0.18, 0.17)),
        material=black,
        name="cabinet_round_fastener",
    )

    # Lower beam fastener row from the reference.
    for i, x in enumerate((-0.31, -0.20, -0.09, 0.09, 0.20, 0.31)):
        frame.visual(
            Cylinder(0.017, 0.012),
            origin=Origin(xyz=(x, -0.164, 0.83), rpy=(math.pi / 2, 0.0, 0.0)),
            material=plate_black,
            name=f"beam_fastener_{i}",
        )

    # Black hydraulic hoses loop from the cylinder over to the right-side control manifold.
    hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.105, -0.005, 2.32),
                (0.32, -0.005, 2.31),
                (0.50, -0.015, 2.20),
                (0.56, -0.020, 1.77),
                (0.66, -0.030, 1.48),
            ],
            radius=0.014,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "top_hydraulic_hose",
    )
    frame.visual(hose_mesh, material=black, name="top_hydraulic_hose")
    return_hose_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.59, -0.015, 1.73),
                (0.52, -0.015, 1.55),
                (0.54, -0.020, 1.18),
                (0.63, -0.025, 0.96),
            ],
            radius=0.010,
            samples_per_segment=12,
            radial_segments=16,
            cap_ends=True,
        ),
        "side_return_hose",
    )
    frame.visual(return_hose_mesh, material=black, name="side_return_hose")
    for name, xyz, radius in (
        ("hose_cylinder_fitting", (0.105, -0.005, 2.32), 0.021),
        ("hose_control_fitting", (0.66, -0.030, 1.48), 0.020),
        ("return_hose_fitting", (0.63, -0.025, 0.96), 0.016),
    ):
        frame.visual(Sphere(radius), origin=Origin(xyz=xyz), material=black, name=name)

    # Sliding blue lower bed/table, separately articulated for height adjustment.
    lower_bed = model.part("lower_bed")
    lower_bed.visual(
        Box((0.72, 0.26, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="blue_bed_block",
    )
    lower_bed.visual(
        Box((0.78, 0.06, 0.065)),
        origin=Origin(xyz=(0.0, -0.155, -0.075)),
        material=plate_black,
        name="front_pin_bar",
    )
    for i, x in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)):
        lower_bed.visual(
            Cylinder(0.015, 0.014),
            origin=Origin(xyz=(x, -0.191, -0.073), rpy=(math.pi / 2, 0.0, 0.0)),
            material=silver,
            name=f"bed_fastener_{i}",
        )
    lower_bed.visual(
        Box((0.038, 0.05, 0.11)),
        origin=Origin(xyz=(-0.379, -0.08, 0.0)),
        material=dark_gray,
        name="bed_guide_sleeve_0",
    )
    lower_bed.visual(
        Box((0.038, 0.05, 0.11)),
        origin=Origin(xyz=(0.379, -0.08, 0.0)),
        material=dark_gray,
        name="bed_guide_sleeve_1",
    )
    lower_bed.visual(
        Box((0.045, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -0.133, -0.005)),
        material=yellow,
        name="bed_small_warning_plate",
    )
    model.articulation(
        "frame_to_lower_bed",
        ArticulationType.FIXED,
        parent=frame,
        child=lower_bed,
        origin=Origin(xyz=(0.0, -0.005, 0.930)),
    )

    # Moving ram and upper platen, with black/yellow hazard stripes on the front face.
    ram = model.part("ram")
    ram.visual(
        Cylinder(0.045, 0.30),
        # The polished rod reaches the fixed gland at q=0 so the moving head
        # reads as captured by the hydraulic cylinder rather than floating.
        origin=Origin(xyz=(0.0, -0.01, -0.12)),
        material=silver,
        name="polished_ram_rod",
    )
    ram.visual(
        Cylinder(0.045, 0.62),
        origin=Origin(xyz=(0.0, -0.01, -0.12)),
        material=silver,
        name="telescoping_ram_sleeve",
    )
    ram.visual(
        Cylinder(0.082, 0.075),
        origin=Origin(xyz=(0.0, -0.01, -0.225)),
        material=blue,
        name="blue_ram_collar",
    )
    ram.visual(
        Box((0.30, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, -0.01, -0.285)),
        material=blue,
        name="ram_head",
    )
    ram.visual(
        Box((0.46, 0.09, 0.060)),
        origin=Origin(xyz=(0.0, -0.01, -0.43)),
        material=yellow,
        name="striped_platen",
    )
    for i, x in enumerate((-0.15, 0.15)):
        ram.visual(
            Cylinder(0.010, 0.055),
            origin=Origin(xyz=(x, -0.01, -0.3725)),
            material=dark_gray,
            name=f"platen_hanger_{i}",
        )
    for i, x in enumerate((-0.18, -0.08, 0.02, 0.12)):
        ram.visual(
            Box((0.12, 0.012, 0.026)),
            origin=Origin(xyz=(x, -0.060, -0.43), rpy=(0.0, -0.45, 0.0)),
            material=plate_black,
            name=f"platen_black_stripe_{i}",
        )
    model.articulation(
        "press_stroke",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.0, -0.01, 1.66)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20000.0, velocity=0.08, lower=0.0, upper=0.14),
    )

    # Depressible front controls on the side control panel.
    button_specs = [
        ("button_0", 1.47, green, 0.020, 0.020),
        ("button_1", 1.36, white, 0.017, 0.018),
        ("button_2", 1.25, orange, 0.031, 0.024),
        ("button_3", 1.13, red, 0.027, 0.023),
        ("button_4", 1.01, white, 0.024, 0.021),
    ]
    control_front_y = -0.111
    for name, z, mat, radius, length in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius, length),
            origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        if name == "button_2":
            button.visual(
                Cylinder(radius * 0.62, length + 0.004),
                origin=Origin(xyz=(0.0, 0.0, 0.002), rpy=(math.pi / 2, 0.0, 0.0)),
                material=plate_black,
                name="emergency_center",
            )
        model.articulation(
            f"control_to_{name}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=button,
            origin=Origin(xyz=(0.72, control_front_y - length / 2.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.04, lower=0.0, upper=0.012),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    ram = object_model.get_part("ram")
    lower_bed = object_model.get_part("lower_bed")
    stroke = object_model.get_articulation("press_stroke")
    bed_mount = object_model.get_articulation("frame_to_lower_bed")

    required_frame_visuals = [
        "top_crossbeam",
        "lower_crossbeam",
        "hydraulic_cylinder",
        "control_box",
        "top_hydraulic_hose",
        "side_return_hose",
        "guide_column_0",
        "guide_column_1",
    ]
    for visual_name in required_frame_visuals:
        ctx.check(
            f"frame has {visual_name}",
            frame.get_visual(visual_name) is not None,
            details=f"Missing frame visual {visual_name}",
        )

    ctx.check(
        "press stroke is prismatic and non-fixed",
        stroke.articulation_type == ArticulationType.PRISMATIC,
        details=f"stroke type={stroke.articulation_type}",
    )
    ctx.check(
        "lower bed is fixed on the supported lower crossbeam",
        bed_mount.articulation_type == ArticulationType.FIXED,
        details=f"bed type={bed_mount.articulation_type}",
    )
    ctx.check(
        "ram includes striped upper platen",
        ram.get_visual("striped_platen") is not None
        and ram.get_visual("platen_black_stripe_0") is not None,
        details="The moving ram must carry the black-and-yellow pressing plate.",
    )
    ctx.check(
        "lower bed is large blue table",
        lower_bed.get_visual("blue_bed_block") is not None
        and lower_bed.get_visual("front_pin_bar") is not None,
        details="The adjustable lower bed/table and its black fastener bar are required.",
    )

    ctx.expect_overlap(
        ram,
        frame,
        axes="xy",
        elem_a="polished_ram_rod",
        elem_b="ram_gland",
        min_overlap=0.030,
        name="ram rod remains aligned under hydraulic cylinder",
    )
    ctx.expect_overlap(
        lower_bed,
        frame,
        axes="x",
        elem_a="blue_bed_block",
        elem_b="lower_crossbeam",
        min_overlap=0.60,
        name="lower bed spans the frame opening",
    )
    ctx.expect_gap(
        lower_bed,
        frame,
        axis="z",
        positive_elem="blue_bed_block",
        negative_elem="lower_crossbeam",
        min_gap=-0.02,
        max_gap=0.02,
        name="lower bed sits directly on supported lower crossbeam",
    )
    ctx.expect_gap(
        ram,
        lower_bed,
        axis="z",
        positive_elem="striped_platen",
        negative_elem="blue_bed_block",
        min_gap=0.16,
        max_gap=0.22,
        name="retracted upper platen clears lower bed",
    )

    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({stroke: 0.13}):
        stroked_ram = ctx.part_world_position(ram)
        ctx.expect_gap(
            ram,
            lower_bed,
            axis="z",
            positive_elem="striped_platen",
            negative_elem="blue_bed_block",
            min_gap=0.03,
            max_gap=0.08,
            name="press stroke lowers platen toward lower bed without collision",
        )
    ctx.check(
        "positive press stroke moves downward",
        rest_ram is not None and stroked_ram is not None and stroked_ram[2] < rest_ram[2] - 0.12,
        details=f"rest={rest_ram}, stroked={stroked_ram}",
    )

    button_joint = object_model.get_articulation("control_to_button_2")
    button = object_model.get_part("button_2")
    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.010}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "emergency control depresses inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] > rest_button[1] + 0.008,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
