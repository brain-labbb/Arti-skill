from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    section_loft,
    superellipse_profile,
)


def _loop_from_profile(profile: list[tuple[float, float]], z: float, sx: float = 1.0, sy: float = 1.0):
    return tuple((x * sx, y * sy, z) for x, y in profile)


def _trapezoid_profile(bottom_width: float, top_width: float, depth: float) -> list[tuple[float, float]]:
    return [
        (-bottom_width / 2.0, -depth / 2.0),
        (bottom_width / 2.0, -depth / 2.0),
        (top_width / 2.0, depth / 2.0),
        (-top_width / 2.0, depth / 2.0),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="winged_conference_phone")

    black = model.material("matte_black", rgba=(0.002, 0.002, 0.002, 1.0))
    soft_black = model.material("soft_rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    grille_black = model.material("woven_black_grille", rgba=(0.035, 0.034, 0.032, 1.0))
    silver = model.material("brushed_silver_gray", rgba=(0.62, 0.62, 0.58, 1.0))
    charcoal = model.material("charcoal_insert", rgba=(0.025, 0.025, 0.026, 1.0))
    lcd_green = model.material("muted_lcd_green", rgba=(0.36, 0.43, 0.34, 1.0))
    off_white = model.material("warm_white_keys", rgba=(0.88, 0.86, 0.82, 1.0))
    key_black = model.material("black_keys", rgba=(0.006, 0.006, 0.006, 1.0))
    label_dark = model.material("printed_dark_labels", rgba=(0.0, 0.0, 0.0, 1.0))
    label_light = model.material("pale_key_legends", rgba=(0.78, 0.78, 0.74, 1.0))
    green = model.material("green_call_key", rgba=(0.0, 0.48, 0.17, 1.0))
    red = model.material("red_end_key", rgba=(0.72, 0.02, 0.02, 1.0))

    body = model.part("body")

    # Broad flattened wing body: wide speaker "wings", narrowed front waist, and
    # a raised curved rear shoulder, at a real table-phone scale.
    outline_points = [
        (-0.294, -0.082),
        (-0.264, -0.137),
        (-0.184, -0.156),
        (-0.086, -0.132),
        (0.0, -0.118),
        (0.086, -0.132),
        (0.184, -0.156),
        (0.264, -0.137),
        (0.294, -0.082),
        (0.284, 0.034),
        (0.220, 0.112),
        (0.110, 0.150),
        (0.0, 0.160),
        (-0.110, 0.150),
        (-0.220, 0.112),
        (-0.284, 0.034),
    ]
    smooth_outline = sample_catmull_rom_spline_2d(outline_points, samples_per_segment=5, closed=True)
    body_shell = section_loft(
        [
            _loop_from_profile(smooth_outline, 0.000, 1.00, 1.00),
            _loop_from_profile(smooth_outline, 0.012, 1.00, 0.99),
            _loop_from_profile(smooth_outline, 0.027, 0.965, 0.930),
            _loop_from_profile(smooth_outline, 0.034, 0.905, 0.850),
        ],
        repair="mesh",
    )
    body.visual(mesh_from_geometry(body_shell, "winged_body_shell"), material=black, name="winged_body_shell")

    # Oval speaker pods with raised rubber surrounds and perforated/textured grille faces.
    speaker_surround = ExtrudeGeometry.from_z0(superellipse_profile(0.205, 0.148, 2.15, segments=64), 0.006)
    speaker_grille = PerforatedPanelGeometry(
        (0.178, 0.126),
        0.004,
        hole_diameter=0.0032,
        pitch=(0.0074, 0.0068),
        frame=0.011,
        corner_radius=0.050,
        stagger=True,
    )
    for suffix, x, yaw in (("0", -0.188, 0.12), ("1", 0.188, -0.12)):
        body.visual(
            mesh_from_geometry(speaker_surround, f"speaker_surround_{suffix}"),
            origin=Origin(xyz=(x, -0.014, 0.030), rpy=(0.0, 0.0, yaw)),
            material=soft_black,
            name=f"speaker_surround_{suffix}",
        )
        body.visual(
            mesh_from_geometry(speaker_grille, f"speaker_grille_{suffix}"),
            origin=Origin(xyz=(x, -0.014, 0.038), rpy=(0.0, 0.0, yaw)),
            material=grille_black,
            name=f"speaker_grille_{suffix}",
        )

    # Raised central trapezoidal silver console with dark recessed operating area.
    console_shell = ExtrudeGeometry.from_z0(_trapezoid_profile(0.180, 0.136, 0.224), 0.020)
    body.visual(
        mesh_from_geometry(console_shell, "silver_console"),
        origin=Origin(xyz=(0.0, 0.002, 0.030)),
        material=silver,
        name="silver_console",
    )
    console_face = ExtrudeGeometry.from_z0(_trapezoid_profile(0.150, 0.112, 0.184), 0.0022)
    body.visual(
        mesh_from_geometry(console_face, "console_face"),
        origin=Origin(xyz=(0.0, -0.005, 0.050)),
        material=charcoal,
        name="console_face",
    )

    display_bezel = BezelGeometry(
        (0.102, 0.042),
        (0.127, 0.064),
        0.005,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.004,
        outer_corner_radius=0.009,
    )
    body.visual(
        mesh_from_geometry(display_bezel, "display_bezel"),
        origin=Origin(xyz=(0.0, 0.064, 0.053)),
        material=key_black,
        name="display_bezel",
    )
    body.visual(
        Box((0.100, 0.040, 0.0012)),
        origin=Origin(xyz=(0.0, 0.064, 0.0554)),
        material=lcd_green,
        name="lcd_screen",
    )
    body.visual(
        Box((0.060, 0.004, 0.001)),
        origin=Origin(xyz=(0.0, 0.108, 0.0504)),
        material=silver,
        name="top_logo_tab",
    )

    # Button cap meshes are reused by many individual articulated pushbutton parts.
    numeric_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.028, 0.019, 0.003, corner_segments=5), 0.0045),
        "numeric_button_cap",
    )
    soft_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.024, 0.014, 0.0025, corner_segments=5), 0.004),
        "soft_button_cap",
    )
    side_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.021, 0.016, 0.0025, corner_segments=5), 0.004),
        "side_button_cap",
    )

    button_limits = MotionLimits(effort=0.7, velocity=0.08, lower=0.0, upper=0.0026)
    button_surface_z = 0.0522

    def add_button(
        name: str,
        *,
        x: float,
        y: float,
        mesh,
        material: Material,
        mark_material: Material,
        mark_size: tuple[float, float, float],
        cap_height: float,
        mark_y: float = 0.0,
    ) -> None:
        part = model.part(name)
        part.visual(mesh, material=material, name="cap")
        part.visual(
            Box(mark_size),
            origin=Origin(xyz=(0.0, mark_y, cap_height + mark_size[2] * 0.25)),
            material=mark_material,
            name="legend_mark",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=part,
            origin=Origin(xyz=(x, y, button_surface_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=button_limits,
        )

    # Twelve white telephone keypad keys in the standard 3 x 4 numeric layout.
    key_xs = (-0.034, 0.0, 0.034)
    key_ys = (-0.022, -0.047, -0.072, -0.097)
    for row, y in enumerate(key_ys):
        for col, x in enumerate(key_xs):
            add_button(
                f"key_{row}_{col}",
                x=x,
                y=y,
                mesh=numeric_cap_mesh,
                material=off_white,
                mark_material=label_dark,
                mark_size=(0.007, 0.0014, 0.00025),
                cap_height=0.0045,
                mark_y=0.0035,
            )

    # Black soft-key row directly below the LCD.
    for index, x in enumerate((-0.056, -0.028, 0.0, 0.028, 0.056)):
        add_button(
            f"soft_key_{index}",
            x=x,
            y=0.006,
            mesh=soft_cap_mesh,
            material=key_black,
            mark_material=label_light,
            mark_size=(0.010, 0.0012, 0.00025),
            cap_height=0.0040,
            mark_y=0.0025,
        )

    # Dedicated call/end and function buttons flank the numeric keypad.
    add_button(
        "call_key",
        x=0.069,
        y=-0.036,
        mesh=side_cap_mesh,
        material=green,
        mark_material=label_light,
        mark_size=(0.008, 0.0015, 0.00025),
        cap_height=0.0040,
    )
    add_button(
        "end_key",
        x=0.069,
        y=-0.064,
        mesh=side_cap_mesh,
        material=red,
        mark_material=label_light,
        mark_size=(0.008, 0.0015, 0.00025),
        cap_height=0.0040,
    )
    for index, y in enumerate((-0.036, -0.064, -0.092)):
        add_button(
            f"function_key_{index}",
            x=-0.069,
            y=y,
            mesh=side_cap_mesh,
            material=key_black,
            mark_material=label_light,
            mark_size=(0.008, 0.0015, 0.00025),
            cap_height=0.0040,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    sample_key = object_model.get_part("key_1_1")
    sample_joint = object_model.get_articulation("body_to_key_1_1")

    ctx.check(
        "black winged trapezoidal body has major visual regions",
        all(
            body.get_visual(name) is not None
            for name in (
                "winged_body_shell",
                "speaker_grille_0",
                "speaker_grille_1",
                "silver_console",
                "display_bezel",
                "lcd_screen",
            )
        ),
        details="Expected body shell, paired speaker grilles, central console, and LCD visuals.",
    )

    ctx.check(
        "individual keypad and soft keys are articulated",
        sum(1 for joint in object_model.articulations if joint.articulation_type == ArticulationType.PRISMATIC) >= 22,
        details="Expected prismatic pushbutton articulations for numeric, soft, call/end, and function keys.",
    )

    ctx.expect_gap(
        sample_key,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0001,
        positive_elem="cap",
        negative_elem="console_face",
        name="key caps sit just proud of the raised console face",
    )

    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: 0.0026}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "pushbutton positive travel depresses downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.002,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
