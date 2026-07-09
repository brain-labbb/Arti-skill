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
                "The reference reads as a clamp-mounted adjustable monitor arm; "
                "no classification mismatch suspected."
            )
        },
    )

    black = model.material("satin_black", rgba=(0.015, 0.017, 0.016, 1.0))
    dark = model.material("dark_hardware", rgba=(0.05, 0.055, 0.052, 1.0))
    silver = model.material("brushed_silver", rgba=(0.58, 0.64, 0.62, 1.0))
    arm_gray = model.material("warm_gray_powdercoat", rgba=(0.56, 0.61, 0.58, 1.0))

    # ── Root: clamp, vertical pole, screw, and fixed hardware ──────────
    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.18, 0.10, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=black,
        name="top_saddle",
    )
    clamp_base.visual(
        Box((0.11, 0.025, 0.018)),
        origin=Origin(xyz=(0.038, 0.038, 0.079), rpy=(0.0, 0.0, 0.22)),
        material=black,
        name="splayed_foot_0",
    )
    clamp_base.visual(
        Box((0.11, 0.025, 0.018)),
        origin=Origin(xyz=(0.038, -0.038, 0.079), rpy=(0.0, 0.0, -0.22)),
        material=black,
        name="splayed_foot_1",
    )
    clamp_base.visual(
        Box((0.045, 0.070, 0.145)),
        origin=Origin(xyz=(-0.055, 0.0, -0.010)),
        material=black,
        name="clamp_spine",
    )
    clamp_base.visual(
        Box((0.130, 0.075, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, -0.090)),
        material=black,
        name="lower_jaw",
    )
    clamp_base.visual(
        Cylinder(radius=0.008, length=0.092),
        origin=Origin(xyz=(-0.055, 0.0, -0.145)),
        material=dark,
        name="clamp_screw",
    )
    clamp_base.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.055, 0.0, -0.195)),
        material=dark,
        name="pressure_pad",
    )
    clamp_base.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(-0.055, 0.0, -0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="thumb_bar",
    )
    clamp_base.visual(
        Cylinder(radius=0.020, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.324)),
        material=silver,
        name="vertical_pole",
    )
    clamp_base.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.487)),
        material=dark,
        name="thrust_collar",
    )

    # ── Single arm segment ─────────────────────────────────────────────
    # One-piece cast arm with swivel sleeve at root, gas-spring linkage
    # underneath, cable clip, and wrist clevis at the tip.  Replaces the
    # parent's two-segment lower_arm + upper_arm + elbow_pitch assembly.
    arm = model.part("arm")
    arm.visual(
        _tube_mesh(0.036, 0.024, 0.092, "base_swivel_sleeve"),
        material=silver,
        name="base_swivel_sleeve",
    )
    arm.visual(
        _rounded_box_mesh(0.470, 0.054, 0.050, 0.006, "arm_shell"),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=arm_gray,
        name="arm_shell",
    )
    arm.visual(
        Box((0.370, 0.018, 0.018)),
        origin=Origin(xyz=(0.230, 0.0, -0.040)),
        material=black,
        name="gas_spring_link",
    )
    arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.080, 0.0, -0.030)),
        material=black,
        name="gas_link_mount_0",
    )
    arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.400, 0.0, -0.030)),
        material=black,
        name="gas_link_mount_1",
    )
    arm.visual(
        Box((0.028, 0.070, 0.008)),
        origin=Origin(xyz=(0.320, 0.0, -0.028)),
        material=dark,
        name="cable_clip",
    )
    arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.530, 0.028, 0.0)),
        material=dark,
        name="wrist_cheek_0",
    )
    arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.530, -0.028, 0.0)),
        material=dark,
        name="wrist_cheek_1",
    )
    arm.visual(
        Cylinder(radius=0.008, length=0.078),
        origin=Origin(xyz=(0.530, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_pin",
    )
    arm.visual(
        Box((0.070, 0.070, 0.016)),
        origin=Origin(xyz=(0.520, 0.0, -0.034)),
        material=dark,
        name="wrist_yoke_bridge",
    )

    # ── Wrist head: tilt knuckle carrying the VESA rotation bearing ────
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

    # ── VESA plate: upright plate with bearing boss and screw pattern ──
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

    # ── Articulations ──────────────────────────────────────────────────
    base_swivel = model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    wrist_tilt = model.articulation(
        "wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=wrist_head,
        origin=Origin(xyz=(0.530, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    vesa_rotation = model.articulation(
        "vesa_rotation",
        ArticulationType.REVOLUTE,
        parent=wrist_head,
        child=vesa_plate,
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    _ = (base_swivel, wrist_tilt, vesa_rotation)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")
    arm = object_model.get_part("arm")
    wrist_head = object_model.get_part("wrist_head")
    vesa_plate = object_model.get_part("vesa_plate")
    base_swivel = object_model.get_articulation("base_swivel")
    wrist_tilt = object_model.get_articulation("wrist_tilt")
    vesa_rotation = object_model.get_articulation("vesa_rotation")

    ctx.check(
        "reference classification",
        object_model.meta.get("reference_note", "").startswith("The reference reads as"),
        details=str(object_model.meta.get("reference_note")),
    )

    # ── Topology: single arm segment, no elbow ─────────────────────────
    part_names = {p.name for p in object_model.parts}
    articulation_names = {a.name for a in object_model.articulations}
    ctx.check(
        "single arm segment exists (lower_arm + upper_arm collapsed into arm)",
        "arm" in part_names
        and "lower_arm" not in part_names
        and "upper_arm" not in part_names,
        details="arm part must exist; lower_arm and upper_arm must be absent",
    )
    ctx.check(
        "elbow_pitch joint removed",
        "elbow_pitch" not in articulation_names,
        details="elbow_pitch articulation must not exist in single-segment variant",
    )

    # ── Structural relationships ───────────────────────────────────────
    ctx.expect_overlap(
        clamp_base,
        arm,
        axes="xy",
        elem_a="vertical_pole",
        elem_b="base_swivel_sleeve",
        min_overlap=0.030,
        name="swivel collar surrounds vertical pole in plan",
    )

    ctx.allow_overlap(
        arm,
        wrist_head,
        elem_a="wrist_pin",
        elem_b="wrist_tilt_eye",
        reason=(
            "The wrist pin is intentionally captured through the tilt eye; the "
            "thin hinge bore is represented by coincident pin-and-eye geometry."
        ),
    )
    ctx.expect_overlap(
        arm,
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

    # ── Articulation behaviour ─────────────────────────────────────────

    # Base swivel swings the single arm about the pole.
    arm_tip_rest = ctx.part_world_position(wrist_head)
    with ctx.pose({base_swivel: 0.75}):
        arm_tip_swiveled = ctx.part_world_position(wrist_head)
    if arm_tip_rest is not None and arm_tip_swiveled is not None:
        rest_y = arm_tip_rest[1]
        swiveled_y = arm_tip_swiveled[1]
        ctx.check(
            "base swivel swings arm about pole",
            abs(swiveled_y - rest_y) > 0.20,
            details=f"rest_y={rest_y:.3f}, swiveled_y={swiveled_y:.3f}",
        )

    # Wrist tilt nods the VESA plate up/down.
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

    # VESA rotation rolls the mounting plate.
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

    # Single-arm geometry: the arm_shell spans from near the swivel sleeve
    # root to the wrist clevis tip as one continuous member.
    arm_shell_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
    sleeve_aabb = ctx.part_element_world_aabb(arm, elem="base_swivel_sleeve")
    wrist_cheek_aabb = ctx.part_element_world_aabb(arm, elem="wrist_cheek_0")
    if arm_shell_aabb is not None and sleeve_aabb is not None and wrist_cheek_aabb is not None:
        shell_reach = arm_shell_aabb[1][0] - arm_shell_aabb[0][0]
        ctx.check(
            "arm shell is one continuous single-segment member",
            shell_reach > 0.40,
            details=f"arm_shell x-span={shell_reach:.3f} m",
        )

    return ctx.report()


object_model = build_object_model()
