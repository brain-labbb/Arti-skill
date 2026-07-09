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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="silver_bullet_cctv_camera")

    silver = model.material("brushed_silver", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.38, 0.38, 0.36, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.035, 0.055, 0.74))
    led_clear = model.material("clear_led_lens", rgba=(0.92, 0.98, 1.0, 0.72))
    led_core = model.material("pale_ir_diode", rgba=(0.86, 0.95, 1.0, 1.0))
    white = model.material("white_powder_coat", rgba=(0.96, 0.96, 0.93, 1.0))
    cable_mat = model.material("warm_gray_cable", rgba=(0.46, 0.46, 0.43, 1.0))
    red = model.material("red_indicator", rgba=(0.8, 0.05, 0.03, 1.0))
    green = model.material("green_indicator", rgba=(0.03, 0.55, 0.18, 1.0))

    # Root: wall/table mounting base.  The base is intentionally a compact
    # round plate with visible screw heads, like a small CCTV stand foot.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=white,
        name="mount_plate",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=white,
        name="center_boss",
    )
    for i, angle in enumerate((math.radians(35), math.radians(145), math.radians(270))):
        base.visual(
            Cylinder(radius=0.0048, length=0.002),
            origin=Origin(
                xyz=(0.039 * math.cos(angle), 0.039 * math.sin(angle), 0.007),
            ),
            material=dark_silver,
            name=f"screw_head_{i}",
        )

    # Swiveling post/yoke assembly.  It carries the camera on a side-to-side
    # pitch pin and can yaw on the base.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.012, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=white,
        name="upright_post",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=white,
        name="post_collar",
    )
    stand.visual(
        Box((0.047, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=white,
        name="yoke_bridge",
    )
    stand.visual(
        Box((0.047, 0.009, 0.046)),
        origin=Origin(xyz=(0.0, -0.025, 0.105)),
        material=white,
        name="yoke_cheek_0",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, -0.02525, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_silver,
        name="pivot_washer_0",
    )
    stand.visual(
        Box((0.047, 0.009, 0.046)),
        origin=Origin(xyz=(0.0, 0.025, 0.105)),
        material=white,
        name="yoke_cheek_1",
    )
    stand.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.02525, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_silver,
        name="pivot_washer_1",
    )
    stand.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, -0.034, 0.125), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="thumb_knob",
    )

    # Camera frame origin is the pitch pin center.  At q=0 the bullet points
    # along +X; positive pitch raises the lens end.
    camera = model.part("camera")
    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    camera.visual(
        Cylinder(radius=0.033, length=0.150),
        origin=Origin(xyz=(0.045, 0.0, 0.045), rpy=cyl_x.rpy),
        material=silver,
        name="cylindrical_body",
    )
    camera.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(-0.034, 0.0, 0.045), rpy=cyl_x.rpy),
        material=dark_silver,
        name="rear_cap",
    )
    # Widened front bezel for dual-sensor (multi-lens) bullet layout.
    camera.visual(
        Box((0.024, 0.088, 0.066)),
        origin=Origin(xyz=(0.124, 0.0, 0.045)),
        material=silver,
        name="front_bezel_shell",
    )
    # Shared black front board behind both lens modules.
    camera.visual(
        Box((0.005, 0.082, 0.060)),
        origin=Origin(xyz=(0.137, 0.0, 0.045)),
        material=black,
        name="black_led_board",
    )

    # Two side-by-side lens+IR modules emitted via a shared helper loop.
    lens_module_count = 2
    module_y_offsets = (-0.022, 0.022)

    def _add_lens_module(cam, idx: int, y_center: float, x_rot: Origin) -> None:
        """Emit one lens barrel + glass + reflection + IR LED ring."""
        z_center = 0.045
        # Silver bezel ring surrounding this module
        cam.visual(
            Cylinder(radius=0.021, length=0.003),
            origin=Origin(xyz=(0.1355, y_center, z_center), rpy=x_rot.rpy),
            material=silver,
            name=f"module_bezel_ring_{idx}",
        )
        # Lens barrel (black stepped barrel)
        cam.visual(
            Cylinder(radius=0.012, length=0.013),
            origin=Origin(xyz=(0.141, y_center, z_center), rpy=x_rot.rpy),
            material=black,
            name=f"lens_barrel_{idx}",
        )
        # Smoked front lens glass
        cam.visual(
            Cylinder(radius=0.009, length=0.005),
            origin=Origin(xyz=(0.149, y_center, z_center), rpy=x_rot.rpy),
            material=glass,
            name=f"front_lens_glass_{idx}",
        )
        # Small reflective pupil
        cam.visual(
            Sphere(radius=0.004),
            origin=Origin(xyz=(0.152, y_center, z_center)),
            material=glass,
            name=f"lens_reflection_{idx}",
        )
        # Infrared LED ring: each LED has a base pad and clear domed cap
        led_count = 14
        for j in range(led_count):
            angle = 2.0 * math.pi * j / led_count + math.pi / led_count
            ly = y_center + 0.017 * math.cos(angle)
            lz = z_center + 0.017 * math.sin(angle)
            cam.visual(
                Cylinder(radius=0.003, length=0.002),
                origin=Origin(xyz=(0.139, ly, lz), rpy=x_rot.rpy),
                material=led_core,
                name=f"led_pad_{idx}_{j}",
            )
            cam.visual(
                Sphere(radius=0.003),
                origin=Origin(xyz=(0.142, ly, lz)),
                material=led_clear,
                name=f"ir_led_{idx}_{j}",
            )

    for i in range(lens_module_count):
        _add_lens_module(camera, i, module_y_offsets[i], cyl_x)

    # Small side details and an upper sun lip give the simple bullet body a
    # manufactured CCTV-camera silhouette without overcomplicating it.
    camera.visual(
        Box((0.085, 0.0025, 0.010)),
        origin=Origin(xyz=(0.047, -0.0336, 0.045)),
        material=dark_silver,
        name="side_label_strip",
    )
    camera.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.080, -0.0360, 0.049)),
        material=red,
        name="red_side_dot",
    )
    camera.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.087, -0.0360, 0.049)),
        material=green,
        name="green_side_dot",
    )
    camera.visual(
        Box((0.112, 0.045, 0.004)),
        origin=Origin(xyz=(0.056, 0.0, 0.079)),
        material=silver,
        name="top_sun_lip",
    )
    camera.visual(
        Box((0.016, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=silver,
        name="lower_mount_lug",
    )
    camera.visual(
        Cylinder(radius=0.0058, length=0.041),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_silver,
        name="pitch_pin",
    )

    cable_path = tube_from_spline_points(
        [
            (-0.040, 0.0, 0.045),
            (-0.075, -0.010, 0.045),
            (-0.120, -0.035, 0.040),
            (-0.180, -0.072, 0.032),
        ],
        radius=0.0028,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    camera.visual(
        mesh_from_geometry(cable_path, "rear_trailing_cable"),
        material=cable_mat,
        name="rear_trailing_cable",
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "stand_to_camera",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-0.45, upper=0.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    camera = object_model.get_part("camera")
    yaw = object_model.get_articulation("base_to_stand")
    pitch = object_model.get_articulation("stand_to_camera")

    # Dual-module structure: two lens barrels and two sets of IR LEDs
    lens_barrel_names = [
        v.name for v in camera.visuals if v.name and v.name.startswith("lens_barrel_")
    ]
    ctx.check(
        "front has two side-by-side lens modules",
        len(lens_barrel_names) == 2,
        details=f"found {len(lens_barrel_names)} lens barrels: {lens_barrel_names}",
    )
    led_names = [v.name for v in camera.visuals if v.name and v.name.startswith("ir_led_")]
    ctx.check(
        "front has infrared leds for both modules",
        len(led_names) >= 24,
        details=f"found {len(led_names)} LED domes",
    )
    for i in range(2):
        ctx.expect_overlap(
            camera,
            camera,
            axes="yz",
            elem_a=f"front_lens_glass_{i}",
            elem_b="black_led_board",
            min_overlap=0.010,
            name=f"lens module {i} sits within the front LED board",
        )
    ctx.expect_overlap(
        camera,
        camera,
        axes="z",
        elem_a="rear_trailing_cable",
        elem_b="rear_cap",
        min_overlap=0.003,
        name="rear cable exits from the cap height",
    )
    ctx.expect_gap(
        camera,
        stand,
        axis="z",
        positive_elem="cylindrical_body",
        negative_elem="yoke_cheek_0",
        min_gap=0.005,
        name="camera body clears the yoke cheeks",
    )

    with ctx.pose({pitch: 0.0, yaw: 0.0}):
        rest_aabb = ctx.part_world_aabb(camera)
    with ctx.pose({pitch: 0.55, yaw: 0.0}):
        raised_aabb = ctx.part_world_aabb(camera)
    ctx.check(
        "positive pitch raises the lens end",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.018,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )
    with ctx.pose({yaw: math.radians(35.0), pitch: 0.0}):
        turned_pos = ctx.part_world_position(camera)
    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        straight_pos = ctx.part_world_position(camera)
    ctx.check(
        "yaw swivel keeps the camera on the stand",
        turned_pos is not None
        and straight_pos is not None
        and abs(turned_pos[2] - straight_pos[2]) < 0.002,
        details=f"straight={straight_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
