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


def _clevis_cheeks_and_pin(
    part,
    *,
    x: float,
    cheek_y: float,
    cheek_size: tuple[float, float, float],
    pin_radius: float,
    pin_length: float,
    bolt_radius: float,
    bolt_length: float,
    bolt_y: float,
    cheek_material,
    pin_material,
    bolt_material,
    prefix: str,
):
    """Add a clevis fork (two cheeks), a through pin, and two bolt heads."""
    part.visual(
        Box(cheek_size),
        origin=Origin(xyz=(x, cheek_y, 0.0)),
        material=cheek_material,
        name=f"{prefix}_cheek_0",
    )
    part.visual(
        Box(cheek_size),
        origin=Origin(xyz=(x, -cheek_y, 0.0)),
        material=cheek_material,
        name=f"{prefix}_cheek_1",
    )
    part.visual(
        Cylinder(radius=pin_radius, length=pin_length),
        origin=Origin(xyz=(x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name=f"{prefix}_pin",
    )
    part.visual(
        Cylinder(radius=bolt_radius, length=bolt_length),
        origin=Origin(xyz=(x, bolt_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_material,
        name=f"{prefix}_bolt_head_0",
    )
    part.visual(
        Cylinder(radius=bolt_radius, length=bolt_length),
        origin=Origin(xyz=(x, -bolt_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_material,
        name=f"{prefix}_bolt_head_1",
    )


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

    # ── Root: clamp, vertical pole, screw, and fixed hardware ──────────────
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

    # ── Lower arm: swivel collar, cast shell, cable clip, elbow clevis ─────
    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        _tube_mesh(0.036, 0.024, 0.092, "base_swivel_sleeve"),
        material=silver,
        name="base_swivel_sleeve",
    )
    lower_arm.visual(
        _rounded_box_mesh(0.240, 0.052, 0.044, 0.010, "lower_arm_shell"),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material=arm_gray,
        name="lower_arm_shell",
    )
    lower_arm.visual(
        Box((0.026, 0.070, 0.008)),
        origin=Origin(xyz=(0.200, 0.0, -0.024)),
        material=dark,
        name="lower_cable_clip",
    )
    for i, y_sign in enumerate((1.0, -1.0)):
        lower_arm.visual(
            Box((0.068, 0.016, 0.036)),
            origin=Origin(xyz=(0.296, y_sign * 0.022, 0.0)),
            material=arm_gray,
            name=f"elbow_fork_arm_{i}",
        )
    _clevis_cheeks_and_pin(
        lower_arm,
        x=0.320,
        cheek_y=0.032,
        cheek_size=(0.074, 0.015, 0.076),
        pin_radius=0.010,
        pin_length=0.095,
        bolt_radius=0.006,
        bolt_length=0.010,
        bolt_y=0.046,
        cheek_material=dark,
        pin_material=dark,
        bolt_material=black,
        prefix="elbow",
    )

    # ── Mid arm: elbow eye, cast shell, cable clip, mid-fold clevis ────────
    mid_arm = model.part("mid_arm")
    mid_arm.visual(
        _tube_mesh(0.027, 0.012, 0.035, "mid_elbow_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="mid_elbow_eye",
    )
    mid_arm.visual(
        _rounded_box_mesh(0.220, 0.050, 0.046, 0.011, "mid_arm_shell"),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=arm_gray,
        name="mid_arm_shell",
    )
    mid_arm.visual(
        Box((0.026, 0.066, 0.008)),
        origin=Origin(xyz=(0.190, 0.0, -0.025)),
        material=dark,
        name="mid_cable_clip",
    )
    for i, y_sign in enumerate((1.0, -1.0)):
        mid_arm.visual(
            Box((0.058, 0.015, 0.034)),
            origin=Origin(xyz=(0.271, y_sign * 0.021, 0.0)),
            material=arm_gray,
            name=f"mid_fold_fork_arm_{i}",
        )
    _clevis_cheeks_and_pin(
        mid_arm,
        x=0.300,
        cheek_y=0.030,
        cheek_size=(0.068, 0.014, 0.070),
        pin_radius=0.009,
        pin_length=0.088,
        bolt_radius=0.006,
        bolt_length=0.010,
        bolt_y=0.044,
        cheek_material=dark,
        pin_material=dark,
        bolt_material=black,
        prefix="mid_fold",
    )

    # ── Upper arm: mid-fold eye, gas-spring shell, linkage, wrist clevis ───
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _tube_mesh(0.025, 0.011, 0.032, "mid_fold_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="mid_fold_eye",
    )
    upper_arm.visual(
        _rounded_box_mesh(0.260, 0.054, 0.052, 0.012, "upper_arm_shell"),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=arm_gray,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.220, 0.018, 0.018)),
        origin=Origin(xyz=(0.155, 0.0, -0.047)),
        material=black,
        name="gas_spring_link",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.060, 0.0, -0.032)),
        material=black,
        name="gas_link_mount_0",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.255, 0.0, -0.032)),
        material=black,
        name="gas_link_mount_1",
    )
    upper_arm.visual(
        Box((0.028, 0.070, 0.008)),
        origin=Origin(xyz=(0.200, 0.0, -0.030)),
        material=dark,
        name="upper_cable_clip",
    )
    for i, y_sign in enumerate((1.0, -1.0)):
        upper_arm.visual(
            Box((0.058, 0.014, 0.036)),
            origin=Origin(xyz=(0.311, y_sign * 0.022, 0.0)),
            material=arm_gray,
            name=f"wrist_fork_arm_{i}",
        )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.340, 0.028, 0.0)),
        material=dark,
        name="wrist_cheek_0",
    )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.340, -0.028, 0.0)),
        material=dark,
        name="wrist_cheek_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.008, length=0.078),
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_pin",
    )
    upper_arm.visual(
        Box((0.050, 0.050, 0.008)),
        origin=Origin(xyz=(0.290, 0.0, -0.016)),
        material=dark,
        name="wrist_yoke_bridge",
    )

    # ── Wrist head: tilt knuckle carrying VESA rotation bearing ────────────
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

    # ── VESA plate: upright plate with bearing boss and screw pattern ──────
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

    # ── Articulations ──────────────────────────────────────────────────────
    base_swivel = model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    elbow_pitch = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=mid_arm,
        origin=Origin(xyz=(0.320, 0.0, 0.0), rpy=(0.0, -0.30, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.75, upper=0.85),
    )
    mid_fold = model.articulation(
        "mid_fold",
        ArticulationType.REVOLUTE,
        parent=mid_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(0.0, -0.25, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.0, lower=-0.70, upper=0.80),
    )
    wrist_tilt = model.articulation(
        "wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist_head,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    vesa_rotation = model.articulation(
        "vesa_rotation",
        ArticulationType.REVOLUTE,
        parent=wrist_head,
        child=vesa_plate,
        # Counter-pitch so the plate reads upright while the three-segment arm
        # angles upward across elbow + mid-fold rest pitches.
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, 0.55, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    _ = (base_swivel, elbow_pitch, mid_fold, wrist_tilt, vesa_rotation)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    mid_arm = object_model.get_part("mid_arm")
    upper_arm = object_model.get_part("upper_arm")
    wrist_head = object_model.get_part("wrist_head")
    vesa_plate = object_model.get_part("vesa_plate")
    base_swivel = object_model.get_articulation("base_swivel")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    mid_fold = object_model.get_articulation("mid_fold")
    wrist_tilt = object_model.get_articulation("wrist_tilt")
    vesa_rotation = object_model.get_articulation("vesa_rotation")

    ctx.check(
        "reference classification",
        object_model.meta.get("reference_note", "").startswith("The reference reads as"),
        details=str(object_model.meta.get("reference_note")),
    )

    # ── Three-segment structure proof ──────────────────────────────────────
    ctx.check(
        "mid_arm part exists as third arm segment",
        mid_arm is not None,
        details="mid_arm part must be present for three-segment folding arm",
    )
    ctx.check(
        "mid_fold articulation exists",
        mid_fold is not None,
        details="mid_fold revolute must connect mid_arm to upper_arm",
    )

    # ── Swivel collar surrounds pole ──────────────────────────────────────
    ctx.expect_overlap(
        clamp_base,
        lower_arm,
        axes="xy",
        elem_a="vertical_pole",
        elem_b="base_swivel_sleeve",
        min_overlap=0.030,
        name="swivel collar surrounds vertical pole in plan",
    )

    # ── Elbow pin retained in mid-arm eye ─────────────────────────────────
    ctx.expect_overlap(
        lower_arm,
        mid_arm,
        axes="yz",
        elem_a="elbow_pin",
        elem_b="mid_elbow_eye",
        min_overlap=0.015,
        name="elbow eye is retained on the hinge pin",
    )

    # ── Mid-fold pin retained in upper-arm eye ────────────────────────────
    ctx.allow_overlap(
        mid_arm,
        upper_arm,
        elem_a="mid_fold_pin",
        elem_b="mid_fold_eye",
        reason=(
            "The mid-fold pin is intentionally captured through the upper-arm "
            "eye; the thin hinge bore is represented by coincident pin-and-eye "
            "geometry at the mid-fold joint."
        ),
    )
    ctx.expect_overlap(
        mid_arm,
        upper_arm,
        axes="yz",
        elem_a="mid_fold_pin",
        elem_b="mid_fold_eye",
        min_overlap=0.012,
        name="mid-fold eye is retained on the mid-fold pin",
    )

    # ── Wrist pin retained in tilt eye ────────────────────────────────────
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

    # ── VESA bearing seated into plate ────────────────────────────────────
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
    ctx.allow_overlap(
        wrist_head,
        vesa_plate,
        elem_a="rotation_bearing",
        elem_b="center_boss",
        reason=(
            "The VESA center boss protrudes forward from the backplate and is "
            "intentionally nested into the wrist rotation bearing bore as a "
            "captured pivot boss."
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

    # ── Base swivel swings arm ────────────────────────────────────────────
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

    # ── Elbow pitch changes wrist height ──────────────────────────────────
    with ctx.pose({elbow_pitch: -0.45}):
        wrist_low = ctx.part_world_position(wrist_head)
    with ctx.pose({elbow_pitch: 0.45}):
        wrist_high = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow pitch changes wrist height",
        wrist_low is not None
        and wrist_high is not None
        and abs(wrist_high[2] - wrist_low[2]) > 0.10,
        details=f"low={wrist_low}, high={wrist_high}",
    )

    # ── Mid-fold pitch changes wrist height (new articulation proof) ──────
    with ctx.pose({mid_fold: -0.40}):
        wrist_fold_low = ctx.part_world_position(wrist_head)
    with ctx.pose({mid_fold: 0.40}):
        wrist_fold_high = ctx.part_world_position(wrist_head)
    ctx.check(
        "mid_fold pitch changes wrist height",
        wrist_fold_low is not None
        and wrist_fold_high is not None
        and abs(wrist_fold_high[2] - wrist_fold_low[2]) > 0.08,
        details=f"low={wrist_fold_low}, high={wrist_fold_high}",
    )

    # ── Wrist tilt nods VESA plate ────────────────────────────────────────
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

    # ── VESA rotation rolls plate ─────────────────────────────────────────
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
