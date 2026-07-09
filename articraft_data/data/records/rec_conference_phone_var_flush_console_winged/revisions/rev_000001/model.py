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

    # All-black monotone palette: subtle shade variations for depth only.
    black = model.material("matte_black", rgba=(0.002, 0.002, 0.002, 1.0))
    soft_black = model.material("soft_rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    grille_black = model.material("woven_black_grille", rgba=(0.035, 0.034, 0.032, 1.0))
    dark_console = model.material("dark_console_well", rgba=(0.028, 0.028, 0.030, 1.0))
    console_face_mat = model.material("flush_console_face", rgba=(0.042, 0.042, 0.044, 1.0))
    lcd_green = model.material("muted_lcd_green", rgba=(0.36, 0.43, 0.34, 1.0))
    key_dark = model.material("dark_keys", rgba=(0.052, 0.052, 0.050, 1.0))
    key_black = model.material("black_keys", rgba=(0.006, 0.006, 0.006, 1.0))
    label_subtle = model.material("subtle_dark_labels", rgba=(0.018, 0.018, 0.018, 1.0))
    legend_light = model.material("muted_legends", rgba=(0.42, 0.42, 0.40, 1.0))
    call_dark = model.material("dark_green_call", rgba=(0.02, 0.06, 0.025, 1.0))
    end_dark = model.material("dark_red_end", rgba=(0.10, 0.02, 0.02, 1.0))

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

    # Flush control deck recessed into the body top surface (body top at z=0.034).
    # Console well: a dark trapezoidal recess embedded below the body top.
    console_well = ExtrudeGeometry.from_z0(_trapezoid_profile(0.180, 0.136, 0.224), 0.004)
    body.visual(
        mesh_from_geometry(console_well, "console_recess"),
        origin=Origin(xyz=(0.0, 0.002, 0.030)),
        material=dark_console,
        name="console_recess",
    )
    # Console face: flush panel whose top surface is coplanar with the body top.
    console_face = ExtrudeGeometry.from_z0(_trapezoid_profile(0.150, 0.112, 0.184), 0.002)
    body.visual(
        mesh_from_geometry(console_face, "console_face"),
        origin=Origin(xyz=(0.0, -0.005, 0.032)),
        material=console_face_mat,
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
        origin=Origin(xyz=(0.0, 0.064, 0.029)),
        material=key_black,
        name="display_bezel",
    )
    body.visual(
        Box((0.100, 0.040, 0.0012)),
        origin=Origin(xyz=(0.0, 0.064, 0.0316)),
        material=lcd_green,
        name="lcd_screen",
    )
    body.visual(
        Box((0.060, 0.004, 0.001)),
        origin=Origin(xyz=(0.0, 0.082, 0.0324)),
        material=console_face_mat,
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
    # Keys originate at the body top surface (z=0.034) for the flush deck design.
    button_surface_z = 0.034

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

    # Twelve dark telephone keypad keys in the standard 3 x 4 numeric layout.
    key_xs = (-0.034, 0.0, 0.034)
    key_ys = (-0.022, -0.047, -0.072, -0.097)
    for row, y in enumerate(key_ys):
        for col, x in enumerate(key_xs):
            add_button(
                f"key_{row}_{col}",
                x=x,
                y=y,
                mesh=numeric_cap_mesh,
                material=key_dark,
                mark_material=label_subtle,
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
            mark_material=legend_light,
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
        material=call_dark,
        mark_material=legend_light,
        mark_size=(0.008, 0.0015, 0.00025),
        cap_height=0.0040,
    )
    add_button(
        "end_key",
        x=0.069,
        y=-0.064,
        mesh=side_cap_mesh,
        material=end_dark,
        mark_material=legend_light,
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
            mark_material=legend_light,
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
        "black winged body has flush recessed console deck",
        all(
            body.get_visual(name) is not None
            for name in (
                "winged_body_shell",
                "speaker_grille_0",
                "speaker_grille_1",
                "console_recess",
                "console_face",
                "display_bezel",
                "lcd_screen",
            )
        ),
        details="Expected body shell, paired speaker grilles, recessed console well, flush face, and LCD.",
    )

    ctx.check(
        "individual keypad and soft keys are articulated",
        sum(1 for joint in object_model.articulations if joint.articulation_type == ArticulationType.PRISMATIC) >= 22,
        details="Expected prismatic pushbutton articulations for numeric, soft, call/end, and function keys.",
    )

    # Verify the flush deck: key articulation originates at the body top surface (z=0.034),
    # not at the old raised console level.
    joint_origin_z = sample_joint.origin.xyz[2]
    ctx.check(
        "flush_deck: key articulation originates at body top surface",
        abs(joint_origin_z - 0.034) < 0.001,
        details=f"Joint body_to_key_1_1 origin z={joint_origin_z:.4f}, expected 0.034 (body loft top).",
    )

    ctx.expect_gap(
        sample_key,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0001,
        positive_elem="cap",
        negative_elem="console_face",
        name="key caps sit just proud of the flush console face",
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
