from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_bullet_security_camera")

    white_plastic = model.material("white_plastic", rgba=(0.94, 0.95, 0.94, 1.0))
    satin_white = model.material("satin_white", rgba=(0.88, 0.90, 0.90, 1.0))
    black_face = model.material("black_face", rgba=(0.015, 0.015, 0.017, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.03, 0.05, 0.08, 0.70))
    smoke_glass = model.material("smoke_glass", rgba=(0.18, 0.20, 0.22, 0.55))
    led_clear = model.material("led_clear", rgba=(0.86, 0.90, 0.92, 0.72))
    led_silver = model.material("led_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    screw_black = model.material("screw_black", rgba=(0.02, 0.02, 0.022, 1.0))

    # ── Root: wall plate and support arm ──────────────────────────────────
    bracket = model.part("wall_bracket")
    bracket.visual(
        Box((0.014, 0.150, 0.200)),
        origin=Origin(xyz=(-0.180, 0.0, 0.0)),
        material=white_plastic,
        name="wall_plate",
    )
    bracket.visual(
        Cylinder(radius=0.045, length=0.024),
        origin=Origin(xyz=(-0.163, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="wall_boss",
    )
    # Support arm: cylinder along X, outboard face at world x = -0.032.
    bracket.visual(
        Cylinder(radius=0.018, length=0.126),
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="support_arm",
    )
    for index, (sy, sz) in enumerate(
        ((0.050, 0.068), (-0.050, 0.068), (0.050, -0.068), (-0.050, -0.068))
    ):
        bracket.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(-0.170, sy, sz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_black,
            name=f"wall_screw_{index}",
        )

    # ── Pan swivel collar ────────────────────────────────────────────────
    # Part frame at the arm/collar contact face (world x = -0.032).
    # The collar and knuckle yoke sit on this frame and rotate together
    # about the arm longitudinal axis (X).
    pan_collar = model.part("pan_collar")
    # Swivel collar: short cylinder flush on the arm outboard face.
    pan_collar.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="swivel_collar",
    )
    # Knuckle bridge: connects collar outboard face to the yoke ears.
    pan_collar.visual(
        Box((0.024, 0.088, 0.054)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=white_plastic,
        name="knuckle_bridge",
    )
    # Yoke ears and pin caps (positive Y side).
    pan_collar.visual(
        Box((0.050, 0.011, 0.064)),
        origin=Origin(xyz=(0.064, 0.0335, 0.0)),
        material=white_plastic,
        name="knuckle_ear_pos",
    )
    pan_collar.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.069, 0.0415, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="pin_cap_pos",
    )
    # Yoke ears and pin caps (negative Y side).
    pan_collar.visual(
        Box((0.050, 0.011, 0.064)),
        origin=Origin(xyz=(0.064, -0.0335, 0.0)),
        material=white_plastic,
        name="knuckle_ear_neg",
    )
    pan_collar.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.069, -0.0415, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="pin_cap_neg",
    )
    # Cross pin through the yoke.
    pan_collar.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=screw_black,
        name="knuckle_pin",
    )

    # ── Camera bullet ────────────────────────────────────────────────────
    # Part frame at the pivot centre (coincides with knuckle_pitch joint).
    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white_plastic,
        name="pivot_barrel",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="rear_neck",
    )
    camera.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.072, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_white,
        name="rear_collar",
    )
    camera.visual(
        Cylinder(radius=0.056, length=0.170),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="cylindrical_body",
    )
    camera.visual(
        Cylinder(radius=0.059, length=0.010),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_white,
        name="front_rim",
    )
    camera.visual(
        Box((0.006, 0.078, 0.088)),
        origin=Origin(xyz=(0.253, 0.0, 0.0)),
        material=black_face,
        name="front_face",
    )
    camera.visual(
        Box((0.004, 0.044, 0.030)),
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        material=smoke_glass,
        name="lens_window",
    )
    camera.visual(
        Cylinder(radius=0.020, length=0.005),
        origin=Origin(xyz=(0.262, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_black,
        name="lens_bezel",
    )
    camera.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.267, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="central_lens",
    )
    camera.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.272, 0.0, 0.0)),
        material=smoke_glass,
        name="lens_highlight",
    )
    for index, (ey, ez) in enumerate(
        ((0.025, 0.031), (-0.025, 0.031), (0.025, -0.031), (-0.025, -0.031))
    ):
        camera.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.257, ey, ez), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=led_silver,
            name=f"led_cup_{index}",
        )
        camera.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.262, ey, ez), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=led_clear,
            name=f"led_emitter_{index}",
        )
    for index, (sy, sz) in enumerate(
        ((0.032, 0.041), (-0.032, 0.041), (0.032, -0.041), (-0.032, -0.041))
    ):
        camera.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(xyz=(0.257, sy, sz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_black,
            name=f"face_screw_{index}",
        )

    # ── Articulations ────────────────────────────────────────────────────

    # Pan swivel: rotates the collar + downstream about the arm axis (X).
    # Origin at the arm/collar contact face.
    model.articulation(
        "pan_swivel",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=pan_collar,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.5, upper=1.5),
    )

    # Tilt knuckle: pitches the camera about the pan-collar-local Y axis.
    # Origin at the knuckle pin / pivot centre in pan_collar frame.
    model.articulation(
        "knuckle_pitch",
        ArticulationType.REVOLUTE,
        parent=pan_collar,
        child=camera,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    pan_collar = object_model.get_part("pan_collar")
    camera = object_model.get_part("camera")
    pan_joint = object_model.get_articulation("pan_swivel")
    knuckle = object_model.get_articulation("knuckle_pitch")

    # Intentional overlap: knuckle pin captured through pivot barrel.
    ctx.allow_overlap(
        pan_collar,
        camera,
        elem_a="knuckle_pin",
        elem_b="pivot_barrel",
        reason="The black cross pin is intentionally captured through the white pitch knuckle barrel.",
    )

    # ── Visual presence ──
    for visual_name in (
        "front_face",
        "central_lens",
        "led_emitter_0",
        "led_emitter_1",
        "led_emitter_2",
        "led_emitter_3",
    ):
        ctx.check(
            f"{visual_name} present",
            camera.get_visual(visual_name) is not None,
            details=f"camera is missing {visual_name}",
        )

    # ── Pan collar seating on arm ──
    ctx.expect_contact(
        bracket,
        pan_collar,
        elem_a="support_arm",
        elem_b="swivel_collar",
        name="swivel collar seats flush on support arm face",
    )

    # ── Yoke / pivot interface (now between pan_collar and camera) ──
    ctx.expect_gap(
        pan_collar,
        camera,
        axis="y",
        positive_elem="knuckle_ear_pos",
        negative_elem="pivot_barrel",
        min_gap=0.002,
        max_gap=0.012,
        name="positive yoke ear clears pivot barrel",
    )
    ctx.expect_gap(
        camera,
        pan_collar,
        axis="y",
        positive_elem="pivot_barrel",
        negative_elem="knuckle_ear_neg",
        min_gap=0.002,
        max_gap=0.012,
        name="negative yoke ear clears pivot barrel",
    )
    ctx.expect_overlap(
        pan_collar,
        camera,
        axes="xz",
        elem_a="knuckle_ear_pos",
        elem_b="pivot_barrel",
        min_overlap=0.020,
        name="yoke ear surrounds the pivot zone",
    )
    ctx.expect_within(
        pan_collar,
        camera,
        axes="xz",
        inner_elem="knuckle_pin",
        outer_elem="pivot_barrel",
        margin=0.001,
        name="knuckle pin runs inside pivot barrel",
    )
    ctx.expect_overlap(
        pan_collar,
        camera,
        axes="y",
        elem_a="knuckle_pin",
        elem_b="pivot_barrel",
        min_overlap=0.040,
        name="knuckle pin spans through pivot barrel",
    )

    # ── Pan swivel rotates the camera about the arm axis (X) ──
    rest_led_aabb = ctx.part_element_world_aabb(camera, elem="led_emitter_0")
    rest_led_y = (rest_led_aabb[0][1] + rest_led_aabb[1][1]) * 0.5
    with ctx.pose({pan_joint: 1.2}):
        panned_led_aabb = ctx.part_element_world_aabb(camera, elem="led_emitter_0")
        panned_led_y = (panned_led_aabb[0][1] + panned_led_aabb[1][1]) * 0.5
        ctx.check(
            "pan_swivel rotates camera about arm axis",
            abs(panned_led_y - rest_led_y) > 0.015,
            details=f"rest_led_y={rest_led_y:.4f}, panned_led_y={panned_led_y:.4f}",
        )

    # ── Knuckle pitch moves the camera nose ──
    rest_face_aabb = ctx.part_element_world_aabb(camera, elem="front_face")
    rest_face_z = None
    if rest_face_aabb is not None:
        rest_face_z = (rest_face_aabb[0][2] + rest_face_aabb[1][2]) * 0.5
    with ctx.pose({knuckle: 0.45}):
        pitched_face_aabb = ctx.part_element_world_aabb(camera, elem="front_face")
        pitched_face_z = None
        if pitched_face_aabb is not None:
            pitched_face_z = (pitched_face_aabb[0][2] + pitched_face_aabb[1][2]) * 0.5
        ctx.check(
            "knuckle pitch moves camera nose",
            rest_face_z is not None
            and pitched_face_z is not None
            and pitched_face_z < rest_face_z - 0.050,
            details=f"rest_z={rest_face_z}, pitched_z={pitched_face_z}",
        )

    return ctx.report()


object_model = build_object_model()
