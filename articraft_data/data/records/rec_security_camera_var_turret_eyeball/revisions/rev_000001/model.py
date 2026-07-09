from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
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

# Eyeball dome radius (the primary variant axis)
EYEBALL_R = 0.045


def _housing_shell_mesh():
    """Compact puck body with a shallow socket ring bore at the base.

    The socket bore inner radius (0.060) clears the eyeball dome (0.045)
    with enough room for the tilt yoke arms to pass through.
    """
    return LatheGeometry.from_shell_profiles(
        [
            (0.074, 0.000),
            (0.080, 0.025),
            (0.094, 0.060),
            (0.098, 0.105),
            (0.096, 0.155),
            (0.086, 0.207),
        ],
        [
            (0.070, 0.000),
            (0.070, 0.025),
            (0.088, 0.060),
            (0.089, 0.086),
            (0.080, 0.088),
            (0.080, 0.092),
            (0.090, 0.105),
            (0.084, 0.155),
            (0.054, 0.194),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_ptz_eyeball_dome_camera")

    # -- housing (root) -------------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_housing_shell_mesh(), "socket_ring"),
        material=WHITE_PLASTIC,
        name="socket_ring",
    )
    housing.visual(
        Cylinder(radius=0.086, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=WHITE_PLASTIC,
        name="ceiling_plate",
    )
    housing.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=WHITE_PLASTIC,
        name="mount_neck",
    )
    housing.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.073, tube=0.003, radial_segments=12, tubular_segments=72),
            "socket_seam_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=SEAM_GRAY,
        name="lower_lip",
    )
    for index, angle in enumerate(
        (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)
    ):
        housing.visual(
            Cylinder(radius=0.0055, length=0.0035),
            origin=Origin(
                xyz=(0.058 * math.cos(angle), 0.058 * math.sin(angle), 0.223),
            ),
            material=SOFT_GRAY,
            name=f"screw_head_{index}",
        )

    # -- pan carriage ---------------------------------------------------------
    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=WHITE_PLASTIC,
        name="turntable_ring",
    )
    # Yoke arms narrowed to clear the smaller eyeball dome.
    pan_carriage.visual(
        Box((0.010, 0.026, 0.088)),
        origin=Origin(xyz=(-0.052, 0.0, -0.046)),
        material=WHITE_PLASTIC,
        name="side_arm_0",
    )
    pan_carriage.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.050, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PLASTIC,
        name="bearing_0",
    )
    pan_carriage.visual(
        Box((0.010, 0.026, 0.088)),
        origin=Origin(xyz=(0.052, 0.0, -0.046)),
        material=WHITE_PLASTIC,
        name="side_arm_1",
    )
    pan_carriage.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.050, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=WHITE_PLASTIC,
        name="bearing_1",
    )

    # -- camera ball (eyeball mini-dome) --------------------------------------
    camera_ball = model.part("camera_ball")
    # Hemisphere dome facing downward (base at z=0, dome to z=-EYEBALL_R).
    camera_ball.visual(
        mesh_from_geometry(
            DomeGeometry(radius=EYEBALL_R, radial_segments=48, height_segments=16, closed=True),
            "eyeball_dome",
        ),
        origin=Origin(rpy=(math.pi, 0.0, 0.0)),
        material=WHITE_PLASTIC,
        name="eyeball_dome",
    )
    # Tilt pin spans between the two yoke bearings.
    camera_ball.visual(
        Cylinder(radius=0.008, length=0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SOFT_GRAY,
        name="tilt_pin",
    )
    # Front panel + lens stack on the -Y face of the hemisphere.
    camera_ball.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, -0.042, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=WHITE_PLASTIC,
        name="front_panel",
    )
    camera_ball.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.048, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK_RUBBER,
        name="outer_lens_ring",
    )
    camera_ball.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, -0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=SOFT_GRAY,
        name="silver_lens_bezel",
    )
    camera_ball.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, -0.059, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GLASS,
        name="glass_lens",
    )
    camera_ball.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=BLACK_RUBBER,
        name="dark_iris",
    )
    camera_ball.visual(
        Sphere(radius=0.0025),
        origin=Origin(xyz=(-0.003, -0.066, 0.003)),
        material=IR_CLEAR,
        name="lens_highlight",
    )

    # Infrared LED ring on the dome surface around the lens.
    led_positions = [
        (-0.025, -0.006),
        (-0.028, -0.016),
        (-0.028, -0.026),
        (-0.025, -0.034),
        (0.025, -0.006),
        (0.028, -0.016),
        (0.028, -0.026),
        (0.025, -0.034),
    ]
    for index, (x, z) in enumerate(led_positions):
        surface_y = -math.sqrt(max(0.0, EYEBALL_R**2 - x**2 - z**2))
        camera_ball.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, surface_y - 0.003, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=IR_CLEAR,
            name=f"ir_led_{index}",
        )
        camera_ball.visual(
            Cylinder(radius=0.0035, length=0.007),
            origin=Origin(xyz=(x, surface_y - 0.006, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=IR_CORE,
            name=f"ir_core_{index}",
        )

    # -- articulations (identical to parent) ----------------------------------
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

    # Turntable ring nests inside the housing bore as a rotating platform.
    ctx.allow_overlap(
        housing,
        pan_carriage,
        elem_a="socket_ring",
        elem_b="turntable_ring",
        reason="The pan turntable ring rotates inside the housing bore as a nested turntable platform.",
    )

    # Tilt pin captured inside yoke bearings.
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
    # Tilt pin axle passes through the yoke side arms.
    ctx.allow_overlap(
        pan_carriage,
        camera_ball,
        elem_a="side_arm_0",
        elem_b="tilt_pin",
        reason="The tilt pin axle passes through the yoke arm bore as a real pivot shaft.",
    )
    ctx.allow_overlap(
        pan_carriage,
        camera_ball,
        elem_a="side_arm_1",
        elem_b="tilt_pin",
        reason="The tilt pin axle passes through the yoke arm bore as a real pivot shaft.",
    )

    # -- variant axis: hemisphere dome seated in shallow socket ring ----------
    ctx.expect_gap(
        housing,
        camera_ball,
        axis="z",
        min_gap=-0.002,
        max_gap=0.020,
        positive_elem="socket_ring",
        negative_elem="eyeball_dome",
        name="eyeball dome sits within the shallow socket ring",
    )
    ctx.expect_within(
        camera_ball,
        housing,
        axes="xy",
        inner_elem="eyeball_dome",
        outer_elem="socket_ring",
        margin=0.005,
        name="eyeball dome is contained within the socket ring bore in XY",
    )

    # Hemisphere geometry claim: dome extends below the tilt equator but
    # the eyeball_dome top stays near the equator plane (z=0 world).
    dome_aabb = ctx.part_element_world_aabb(camera_ball, elem="eyeball_dome")
    ctx.check(
        "eyeball_dome is a hemisphere below the tilt equator",
        dome_aabb is not None
        and dome_aabb[1][2] <= 0.005
        and dome_aabb[0][2] <= -0.035,
        details=f"eyeball_dome AABB min={dome_aabb[0] if dome_aabb else None}, max={dome_aabb[1] if dome_aabb else None}",
    )

    # Pan ring clearance above the dome equator — yoke arms separate the
    # turntable from the small eyeball dome by a visible standoff gap.
    ctx.expect_gap(
        pan_carriage,
        camera_ball,
        axis="z",
        min_gap=0.020,
        max_gap=0.120,
        positive_elem="turntable_ring",
        negative_elem="eyeball_dome",
        name="pan ring rides above the eyeball dome with yoke standoff",
    )

    # Tilt pin alignment with bearings and passage through yoke arms.
    ctx.expect_overlap(
        pan_carriage,
        camera_ball,
        axes="yz",
        min_overlap=0.008,
        elem_a="bearing_1",
        elem_b="tilt_pin",
        name="tilt pin aligns with the side bearing",
    )
    ctx.expect_overlap(
        pan_carriage,
        camera_ball,
        axes="yz",
        min_overlap=0.010,
        elem_a="side_arm_0",
        elem_b="tilt_pin",
        name="tilt pin passes through the yoke arm bore",
    )

    # Turntable stays centered inside the housing bore.
    ctx.expect_within(
        pan_carriage,
        housing,
        axes="xy",
        inner_elem="turntable_ring",
        outer_elem="socket_ring",
        margin=0.002,
        name="turntable ring stays centered inside the housing bore",
    )

    # IR LED count.
    ctx.check(
        "infrared LED ring has eight lamps",
        sum(1 for v in camera_ball.visuals if v.name and v.name.startswith("ir_led_")) == 8,
        details="Expected eight distinct IR LED lenses around the central lens.",
    )

    # Joint limits preserved from parent.
    ctx.check(
        "pan and tilt joints have realistic speed-dome limits",
        pan.motion_limits.lower <= -3.0
        and pan.motion_limits.upper >= 3.0
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.9,
        details=f"pan={pan.motion_limits}, tilt={tilt.motion_limits}",
    )

    # Decisive pose checks: pan yaws and tilt pitches the front lens.
    def _elem_center(part_obj, elem_name):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
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
        "positive pan yaws the front lens around the housing",
        rest_lens is not None and panned_lens is not None and panned_lens[0] > rest_lens[0] + 0.030,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )
    ctx.check(
        "positive tilt pitches the eyeball lens face downward",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] < rest_lens[2] - 0.020,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
