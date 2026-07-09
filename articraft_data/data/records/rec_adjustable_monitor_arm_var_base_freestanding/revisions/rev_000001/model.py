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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box_mesh(length: float, width: float, height: float, radius: float, name: str):
    """Small rounded rectangular extrusion used for the cast arm shells."""
    fillet = min(radius, length * 0.18, width * 0.42, height * 0.42)
    shape = cq.Workplane("XY").box(length, width, height).edges().fillet(fillet)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _oval_plate_mesh(rx: float, ry: float, thickness: float, name: str):
    """Extruded elliptical plate centered on Z, used for the freestanding weighted base."""
    shape = (
        cq.Workplane("XY")
        .ellipse(rx, ry)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _tapered_hub_mesh(top_radius: float, bottom_radius: float, height: float, name: str):
    """Tapered cylindrical hub centered on Z, used as the pole-receiver on the base."""
    shape = (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft()
        .translate((0.0, 0.0, -height / 2.0))
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.06)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Closed annular tube centered on local Z, used for swivel and hinge eyes."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    cutter = cq.Workplane("XY").circle(inner_radius).extrude(length * 3.0).translate(
        (0.0, 0.0, -length)
    )
    shape = outer.cut(cutter).translate((0.0, 0.0, -length / 2.0))
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.06)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="adjustable_monitor_arm",
        meta={
            "reference_note": (
                "Variant: freestanding weighted-base adjustable monitor arm; "
                "the C-clamp has been replaced with a broad oval base plate."
            )
        },
    )

    black = model.material("satin_black", rgba=(0.015, 0.017, 0.016, 1.0))
    dark = model.material("dark_hardware", rgba=(0.05, 0.055, 0.052, 1.0))
    silver = model.material("brushed_silver", rgba=(0.58, 0.64, 0.62, 1.0))
    arm_gray = model.material("warm_gray_powdercoat", rgba=(0.56, 0.61, 0.58, 1.0))

    # Root: freestanding weighted base plate with vertical pole and thrust collar.
    # The broad oval base rests on the desktop without clamping; a tapered hub
    # receives the pole.  Dimensions match a real monitor-arm weighted base:
    # ~280 x 200 mm footprint, 50 cm pole.
    weighted_base = model.part("weighted_base")
    weighted_base.visual(
        _oval_plate_mesh(0.140, 0.100, 0.022, "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=black,
        name="base_plate",
    )
    weighted_base.visual(
        _tapered_hub_mesh(0.022, 0.034, 0.052, "pole_hub"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=black,
        name="pole_hub",
    )
    for i, (x, y) in enumerate(
        ((0.080, 0.058), (0.080, -0.058), (-0.080, 0.058), (-0.080, -0.058))
    ):
        weighted_base.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(x, y, -0.001)),
            material=dark,
            name=f"rubber_foot_{i}",
        )
    weighted_base.visual(
        Cylinder(radius=0.020, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.324)),
        material=silver,
        name="vertical_pole",
    )
    weighted_base.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.487)),
        material=dark,
        name="thrust_collar",
    )

    # Lower arm: a hollow swivel collar around the post, a cast horizontal arm,
    # a cable clip under the arm, and a forked elbow clevis with a visible pin.
    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        _tube_mesh(0.036, 0.024, 0.092, "base_swivel_sleeve"),
        material=silver,
        name="base_swivel_sleeve",
    )
    lower_arm.visual(
        _rounded_box_mesh(0.335, 0.052, 0.044, 0.010, "lower_arm_shell"),
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        material=arm_gray,
        name="lower_arm_shell",
    )
    lower_arm.visual(
        Box((0.026, 0.070, 0.008)),
        origin=Origin(xyz=(0.235, 0.0, -0.024)),
        material=dark,
        name="lower_cable_clip",
    )
    lower_arm.visual(
        Box((0.080, 0.015, 0.076)),
        origin=Origin(xyz=(0.400, 0.032, 0.0)),
        material=dark,
        name="elbow_cheek_0",
    )
    lower_arm.visual(
        Box((0.080, 0.015, 0.076)),
        origin=Origin(xyz=(0.400, -0.032, 0.0)),
        material=dark,
        name="elbow_cheek_1",
    )
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.095),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_pin",
    )
    lower_arm.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.400, 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="elbow_bolt_head_0",
    )
    lower_arm.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.400, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="elbow_bolt_head_1",
    )

    # Upper arm: angled gas-spring arm shell, exposed lower linkage, cable clip,
    # elbow eye and wrist clevis.  The child frame is pitched at the elbow so the
    # rest pose matches the upward segment in the reference image.
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _tube_mesh(0.027, 0.012, 0.035, "elbow_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_eye",
    )
    upper_arm.visual(
        _rounded_box_mesh(0.370, 0.054, 0.052, 0.012, "upper_arm_shell"),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=arm_gray,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.310, 0.018, 0.018)),
        origin=Origin(xyz=(0.220, 0.0, -0.047)),
        material=black,
        name="gas_spring_link",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.080, 0.0, -0.032)),
        material=black,
        name="gas_link_mount_0",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.350, 0.0, -0.032)),
        material=black,
        name="gas_link_mount_1",
    )
    upper_arm.visual(
        Box((0.028, 0.070, 0.008)),
        origin=Origin(xyz=(0.280, 0.0, -0.030)),
        material=dark,
        name="upper_cable_clip",
    )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.420, 0.028, 0.0)),
        material=dark,
        name="wrist_cheek_0",
    )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.420, -0.028, 0.0)),
        material=dark,
        name="wrist_cheek_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.008, length=0.078),
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_pin",
    )
    upper_arm.visual(
        Box((0.070, 0.070, 0.010)),
        origin=Origin(xyz=(0.410, 0.0, -0.029)),
        material=dark,
        name="wrist_yoke_bridge",
    )

    # Wrist head: short tilt knuckle that carries the VESA rotation bearing.
    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        _tube_mesh(0.022, 0.008, 0.028, "wrist_tilt_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_tilt_eye",
    )
    wrist_head.visual(
        _rounded_box_mesh(0.070, 0.036, 0.036, 0.007, "wrist_neck"),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=black,
        name="wrist_neck",
    )
    wrist_head.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="rotation_bearing",
    )

    # VESA plate: upright plate with a central bearing boss, four screw heads
    # on a 75/100 mm style pattern, rounded corner ears, and black stamped steel
    # proportions similar to the reference.
    vesa_plate = model.part("vesa_plate")
    vesa_plate.visual(
        Box((0.010, 0.118, 0.118)),
        material=black,
        name="vesa_backplate",
    )
    for index, (y, z) in enumerate(
        ((0.052, 0.052), (0.052, -0.052), (-0.052, 0.052), (-0.052, -0.052))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"corner_ear_{index}",
        )
    vesa_plate.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="center_boss",
    )
    for index, (y, z) in enumerate(
        ((0.0375, 0.0375), (0.0375, -0.0375), (-0.0375, 0.0375), (-0.0375, -0.0375))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.008, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"vesa_fastener_{index}",
        )

    base_swivel = model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=weighted_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    elbow_pitch = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, -0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.75, upper=0.85),
    )
    wrist_tilt = model.articulation(
        "wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist_head,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    vesa_rotation = model.articulation(
        "vesa_rotation",
        ArticulationType.REVOLUTE,
        parent=wrist_head,
        child=vesa_plate,
        # Counter-pitch the rest pose so the plate is upright while the arm
        # segment remains angled upward.
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, 0.42, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # Keep references alive for clarity when reading the script.
    _ = (base_swivel, elbow_pitch, wrist_tilt, vesa_rotation)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    weighted_base = object_model.get_part("weighted_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    wrist_head = object_model.get_part("wrist_head")
    vesa_plate = object_model.get_part("vesa_plate")
    base_swivel = object_model.get_articulation("base_swivel")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_tilt = object_model.get_articulation("wrist_tilt")
    vesa_rotation = object_model.get_articulation("vesa_rotation")

    # Verify the variant is the weighted-base fork.
    ctx.check(
        "reference classification",
        "weighted-base" in object_model.meta.get("reference_note", ""),
        details=str(object_model.meta.get("reference_note")),
    )

    # Verify the weighted base plate has a broad flat footprint (>200mm x >150mm).
    base_aabb = ctx.part_element_world_aabb(weighted_base, elem="base_plate")
    if base_aabb is not None:
        dx = base_aabb[1][0] - base_aabb[0][0]
        dy = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "weighted base plate has broad oval footprint",
            dx > 0.20 and dy > 0.15,
            details=f"base_plate span: {dx:.3f}m x {dy:.3f}m",
        )
    # Verify the base plate sits at the bottom of the assembly (below the pole).
    ctx.expect_gap(
        weighted_base,
        weighted_base,
        axis="z",
        positive_elem="vertical_pole",
        negative_elem="base_plate",
        max_penetration=0.005,
        name="pole rises above the base plate top surface",
    )

    # Pole/hub connectivity: the hub bridges the plate to the pole.
    ctx.expect_overlap(
        weighted_base,
        weighted_base,
        axes="xy",
        elem_a="pole_hub",
        elem_b="vertical_pole",
        min_overlap=0.010,
        name="pole hub is centered on the vertical pole in plan",
    )

    ctx.expect_overlap(
        weighted_base,
        lower_arm,
        axes="xy",
        elem_a="vertical_pole",
        elem_b="base_swivel_sleeve",
        min_overlap=0.030,
        name="swivel collar surrounds vertical pole in plan",
    )
    ctx.expect_overlap(
        lower_arm,
        upper_arm,
        axes="yz",
        elem_a="elbow_pin",
        elem_b="elbow_eye",
        min_overlap=0.015,
        name="elbow eye is retained on the hinge pin",
    )
    ctx.allow_overlap(
        upper_arm,
        wrist_head,
        elem_a="wrist_pin",
        elem_b="wrist_tilt_eye",
        reason=(
            "The wrist pin is intentionally captured through the tilt eye; the "
            "thin hinge bore is represented by coincident pin-and-eye geometry."
        ),
    )
    ctx.expect_overlap(
        upper_arm,
        wrist_head,
        axes="yz",
        elem_a="wrist_pin",
        elem_b="wrist_tilt_eye",
        min_overlap=0.012,
        name="wrist tilt eye is retained on the wrist pin",
    )
    ctx.allow_overlap(
        wrist_head,
        vesa_plate,
        elem_a="rotation_bearing",
        elem_b="vesa_backplate",
        reason=(
            "The wrist rotation bearing is intentionally seated into the rear of "
            "the thin stamped VESA plate as a local captured mounting boss."
        ),
    )
    ctx.expect_overlap(
        wrist_head,
        vesa_plate,
        axes="yz",
        elem_a="rotation_bearing",
        elem_b="vesa_backplate",
        min_overlap=0.045,
        name="VESA rotation bearing is centered behind the plate",
    )

    lower_rest_aabb = ctx.part_element_world_aabb(lower_arm, elem="elbow_pin")
    with ctx.pose({base_swivel: 0.75}):
        lower_swiveled_aabb = ctx.part_element_world_aabb(lower_arm, elem="elbow_pin")
    if lower_rest_aabb is not None and lower_swiveled_aabb is not None:
        rest_y = (lower_rest_aabb[0][1] + lower_rest_aabb[1][1]) / 2.0
        swiveled_y = (lower_swiveled_aabb[0][1] + lower_swiveled_aabb[1][1]) / 2.0
        ctx.check(
            "base swivel swings arm about pole",
            abs(swiveled_y - rest_y) > 0.20,
            details=f"rest_y={rest_y:.3f}, swiveled_y={swiveled_y:.3f}",
        )

    with ctx.pose({elbow_pitch: -0.45}):
        wrist_low = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 0.45}):
        wrist_high = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch changes wrist height",
        wrist_low is not None
        and wrist_high is not None
        and abs(wrist_high[2] - wrist_low[2]) > 0.12,
        details=f"low={wrist_low}, high={wrist_high}",
    )

    with ctx.pose({wrist_tilt: -0.45}):
        plate_low = ctx.part_world_position(vesa_plate)
    with ctx.pose({wrist_tilt: 0.45}):
        plate_high = ctx.part_world_position(vesa_plate)
    ctx.check(
        "wrist tilt nods the VESA plate",
        plate_low is not None
        and plate_high is not None
        and abs(plate_high[2] - plate_low[2]) > 0.060,
        details=f"low={plate_low}, high={plate_high}",
    )

    plate_rest_aabb = ctx.part_world_aabb(vesa_plate)
    with ctx.pose({vesa_rotation: 0.55}):
        plate_rotated_aabb = ctx.part_world_aabb(vesa_plate)
    if plate_rest_aabb is not None and plate_rotated_aabb is not None:
        rest_height = plate_rest_aabb[1][2] - plate_rest_aabb[0][2]
        rotated_height = plate_rotated_aabb[1][2] - plate_rotated_aabb[0][2]
        ctx.check(
            "VESA rotation rolls the mounting plate",
            abs(rotated_height - rest_height) > 0.020,
            details=f"rest_height={rest_height:.3f}, rotated_height={rotated_height:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
