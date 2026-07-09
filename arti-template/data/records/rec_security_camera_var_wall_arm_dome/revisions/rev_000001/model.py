from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


WHITE_PLASTIC = Material("warm_white_plastic", rgba=(0.94, 0.95, 0.93, 1.0))
SOFT_GRAY = Material("soft_shadow_gray", rgba=(0.72, 0.74, 0.72, 1.0))
SEAM_GRAY = Material("fine_seam_gray", rgba=(0.55, 0.56, 0.56, 1.0))
BLACK_RUBBER = Material("matte_black_lens_trim", rgba=(0.005, 0.005, 0.006, 1.0))
GLASS = Material("smoked_glass", rgba=(0.08, 0.11, 0.13, 0.72))
IR_CLEAR = Material("clear_ir_led_lens", rgba=(0.82, 0.86, 0.88, 0.62))
IR_CORE = Material("ir_diode_core", rgba=(0.72, 0.78, 0.80, 1.0))


def _top_cap_mesh():
    """Thin lathed shell for the truncated-cone speed-dome cover."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.082, 0.210),
            (0.094, 0.207),
            (0.105, 0.165),
            (0.113, 0.105),
            (0.112, 0.088),
        ],
        [
            (0.054, 0.194),
            (0.075, 0.191),
            (0.091, 0.156),
            (0.090, 0.104),
            (0.076, 0.088),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_ptz_speed_dome_camera")

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_top_cap_mesh(), "truncated_top_cap"),
        material=WHITE_PLASTIC,
        name="cap_shell",
    )
    # --- Wall-mount L-arm bracket (replaces ceiling plate + neck) ---
    # Vertical circular wall plate, axis along X (facing wall)
    housing.visual(
        Cylinder(radius=0.058, length=0.014),
        origin=Origin(xyz=(-0.155, 0.0, 0.272), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PLASTIC,
        name="wall_plate",
    )
    # Horizontal arm extending from wall plate to above dome center
    housing.visual(
        Box((0.184, 0.050, 0.042)),
        origin=Origin(xyz=(-0.056, 0.0, 0.272)),
        material=WHITE_PLASTIC,
        name="horizontal_arm",
    )
    # Drop collar flange seated on dome top annular rim (overlaps shell wall)
    housing.visual(
        Cylinder(radius=0.082, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.213)),
        material=WHITE_PLASTIC,
        name="drop_collar",
    )
    # Narrow pendant stem from flange up into arm
    housing.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=WHITE_PLASTIC,
        name="collar_stem",
    )
    housing.visual(
        mesh_from_geometry(TorusGeometry(radius=0.106, tube=0.0035, radial_segments=12, tubular_segments=72), "lower_cap_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=SEAM_GRAY,
        name="lower_lip",
    )
    # Mounting screws on wall plate face (oriented along X)
    for index, (dy, dz) in enumerate([(0.038, 0.038), (-0.038, 0.038), (0.038, -0.038), (-0.038, -0.038)]):
        housing.visual(
            Cylinder(radius=0.0055, length=0.0035),
            origin=Origin(
                xyz=(-0.147, dy, 0.272 + dz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=SOFT_GRAY,
            name=f"screw_head_{index}",
        )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=WHITE_PLASTIC,
        name="turntable_ring",
    )
    pan_carriage.visual(
        Box((0.014, 0.040, 0.128)),
        origin=Origin(xyz=(-0.084, 0.0, -0.066)),
        material=WHITE_PLASTIC,
        name="side_arm_0",
    )
    pan_carriage.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(-0.080, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PLASTIC,
        name="bearing_0",
    )
    pan_carriage.visual(
        Box((0.014, 0.040, 0.128)),
        origin=Origin(xyz=(0.084, 0.0, -0.066)),
        material=WHITE_PLASTIC,
        name="side_arm_1",
    )
    pan_carriage.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.080, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PLASTIC,
        name="bearing_1",
    )

    camera_ball = model.part("camera_ball")
    camera_ball.visual(
        Sphere(radius=0.070),
        origin=Origin(),
        material=WHITE_PLASTIC,
        name="camera_sphere",
    )
    camera_ball.visual(
        Cylinder(radius=0.009, length=0.153),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SOFT_GRAY,
        name="tilt_pin",
    )
    camera_ball.visual(
        Cylinder(radius=0.060, length=0.009),
        origin=Origin(xyz=(0.0, -0.067, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE_PLASTIC,
        name="front_panel",
    )
    camera_ball.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, -0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK_RUBBER,
        name="outer_lens_ring",
    )
    camera_ball.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.0, -0.079, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=SOFT_GRAY,
        name="silver_lens_bezel",
    )
    camera_ball.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, -0.084, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GLASS,
        name="glass_lens",
    )
    camera_ball.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, -0.094, 0.0)),
        material=BLACK_RUBBER,
        name="dark_iris",
    )
    camera_ball.visual(
        Sphere(radius=0.0035),
        origin=Origin(xyz=(-0.005, -0.103, 0.006)),
        material=IR_CLEAR,
        name="lens_highlight",
    )

    led_positions = [
        (-0.043, -0.035),
        (-0.046, -0.012),
        (-0.046, 0.012),
        (-0.043, 0.035),
        (0.043, -0.035),
        (0.046, -0.012),
        (0.046, 0.012),
        (0.043, 0.035),
    ]
    for index, (x, z) in enumerate(led_positions):
        surface_y = -math.sqrt(max(0.0, 0.070**2 - x**2 - z**2))
        camera_ball.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(x, surface_y - 0.003, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=IR_CLEAR,
            name=f"ir_led_{index}",
        )
        camera_ball.visual(
            Cylinder(radius=0.0043, length=0.008),
            origin=Origin(xyz=(x, surface_y - 0.007, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=IR_CORE,
            name=f"ir_core_{index}",
        )

    model.articulation(
        "housing_to_pan_carriage",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pan_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=3.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pan_carriage_to_camera_ball",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=camera_ball,
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.5, lower=-0.55, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pan_carriage = object_model.get_part("pan_carriage")
    camera_ball = object_model.get_part("camera_ball")
    pan = object_model.get_articulation("housing_to_pan_carriage")
    tilt = object_model.get_articulation("pan_carriage_to_camera_ball")

    ctx.allow_overlap(
        pan_carriage,
        camera_ball,
        elem_a="bearing_0",
        elem_b="tilt_pin",
        reason="The tilt pin is intentionally captured inside the side bearing bore.",
    )
    ctx.allow_overlap(
        pan_carriage,
        camera_ball,
        elem_a="bearing_1",
        elem_b="tilt_pin",
        reason="The tilt pin is intentionally captured inside the side bearing bore.",
    )

    ctx.expect_gap(
        housing,
        camera_ball,
        axis="z",
        min_gap=0.004,
        max_gap=0.025,
        positive_elem="cap_shell",
        negative_elem="camera_sphere",
        name="top cap clears the spherical camera module",
    )
    ctx.expect_gap(
        pan_carriage,
        camera_ball,
        axis="z",
        min_gap=0.0005,
        max_gap=0.006,
        positive_elem="turntable_ring",
        negative_elem="camera_sphere",
        name="pan ring rides just above the ball",
    )
    ctx.expect_overlap(
        pan_carriage,
        camera_ball,
        axes="yz",
        min_overlap=0.012,
        elem_a="bearing_1",
        elem_b="tilt_pin",
        name="tilt pin aligns with the side bearing",
    )
    ctx.check(
        "infrared LED ring has eight lamps",
        sum(1 for visual in camera_ball.visuals if visual.name and visual.name.startswith("ir_led_")) == 8,
        details="Expected eight distinct IR LED lenses around the central lens.",
    )
    ctx.check(
        "pan and tilt joints have realistic speed-dome limits",
        pan.motion_limits.lower <= -3.0
        and pan.motion_limits.upper >= 3.0
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.9,
        details=f"pan={pan.motion_limits}, tilt={tilt.motion_limits}",
    )

    # --- Wall-mount L-arm bracket verification ---
    wall_box = ctx.part_element_world_aabb(housing, elem="wall_plate")
    arm_box = ctx.part_element_world_aabb(housing, elem="horizontal_arm")
    collar_box = ctx.part_element_world_aabb(housing, elem="drop_collar")

    ctx.check(
        "wall_plate is offset in -X from dome center for wall mounting",
        wall_box is not None and wall_box[1][0] < -0.10,
        details=f"wall_plate max_x={wall_box[1][0]:.3f}" if wall_box else "wall_plate not found",
    )
    ctx.check(
        "horizontal_arm spans from wall plate toward dome center",
        arm_box is not None and arm_box[0][0] < -0.12 and arm_box[1][0] > -0.01,
        details=f"arm x=[{arm_box[0][0]:.3f}, {arm_box[1][0]:.3f}]" if arm_box else "horizontal_arm not found",
    )
    ctx.check(
        "drop_collar sits at dome top for pendant mount",
        collar_box is not None and collar_box[0][2] < 0.215 and collar_box[1][2] > 0.215,
        details=f"collar z=[{collar_box[0][2]:.3f}, {collar_box[1][2]:.3f}]" if collar_box else "drop_collar not found",
    )

    def _elem_center(part, elem_name):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = _elem_center(camera_ball, "glass_lens")
    with ctx.pose({pan: 1.0}):
        panned_lens = _elem_center(camera_ball, "glass_lens")
    with ctx.pose({tilt: 0.8}):
        tilted_lens = _elem_center(camera_ball, "glass_lens")

    ctx.check(
        "positive pan yaws the front lens around the cap",
        rest_lens is not None and panned_lens is not None and panned_lens[0] > rest_lens[0] + 0.045,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )
    ctx.check(
        "positive tilt pitches the camera face downward",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] < rest_lens[2] - 0.035,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
