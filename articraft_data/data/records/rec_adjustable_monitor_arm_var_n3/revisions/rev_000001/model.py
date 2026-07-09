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

NUM_ARMS = 3
# Stacked pole heights (m) and rest yaw offsets (rad) for a radial fan layout.
ARM_POLE_Z = (0.490, 0.350, 0.210)
ARM_YAW = (0.40, 0.0, -0.40)


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


def _build_arm_chain(model, idx, pole_z, yaw_offset, clamp_base, materials):
    """Emit one complete arm chain (lower_arm → upper_arm → wrist_head → vesa_plate)
    with its four revolute joints.  Mesh asset names are indexed for global
    uniqueness; visual names are per-part and stay stable across instances."""
    black, dark, silver, arm_gray = materials
    s = f"_{idx}"

    # ── Lower arm ────────────────────────────────────────────────────────
    lower_arm = model.part(f"lower_arm{s}")
    lower_arm.visual(
        _tube_mesh(0.036, 0.020, 0.092, f"swivel_sleeve{s}"),
        material=silver,
        name="base_swivel_sleeve",
    )
    lower_arm.visual(
        _rounded_box_mesh(0.335, 0.052, 0.044, 0.010, f"lower_shell{s}"),
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

    # ── Upper arm ────────────────────────────────────────────────────────
    upper_arm = model.part(f"upper_arm{s}")
    upper_arm.visual(
        _tube_mesh(0.027, 0.012, 0.035, f"elbow_eye{s}"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_eye",
    )
    upper_arm.visual(
        _rounded_box_mesh(0.370, 0.054, 0.052, 0.012, f"upper_shell{s}"),
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

    # ── Wrist head ───────────────────────────────────────────────────────
    wrist_head = model.part(f"wrist_head{s}")
    wrist_head.visual(
        _tube_mesh(0.022, 0.008, 0.028, f"wrist_eye{s}"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_tilt_eye",
    )
    wrist_head.visual(
        _rounded_box_mesh(0.070, 0.036, 0.036, 0.007, f"wrist_neck{s}"),
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

    # ── VESA plate ───────────────────────────────────────────────────────
    vesa_plate = model.part(f"vesa_plate{s}")
    vesa_plate.visual(
        Box((0.010, 0.118, 0.118)),
        material=black,
        name="vesa_backplate",
    )
    for ci, (y, z) in enumerate(
        ((0.052, 0.052), (0.052, -0.052), (-0.052, 0.052), (-0.052, -0.052))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"corner_ear_{ci}",
        )
    vesa_plate.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="center_boss",
    )
    for ci, (y, z) in enumerate(
        ((0.0375, 0.0375), (0.0375, -0.0375), (-0.0375, 0.0375), (-0.0375, -0.0375))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.008, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"vesa_fastener_{ci}",
        )

    # ── Articulations ────────────────────────────────────────────────────
    base_swivel = model.articulation(
        f"base_swivel{s}",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, pole_z), rpy=(0.0, 0.0, yaw_offset)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    elbow_pitch = model.articulation(
        f"elbow_pitch{s}",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, -0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.75, upper=0.85),
    )
    wrist_tilt = model.articulation(
        f"wrist_tilt{s}",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist_head,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    vesa_rotation = model.articulation(
        f"vesa_rotation{s}",
        ArticulationType.REVOLUTE,
        parent=wrist_head,
        child=vesa_plate,
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, 0.42, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return {
        "lower_arm": lower_arm,
        "upper_arm": upper_arm,
        "wrist_head": wrist_head,
        "vesa_plate": vesa_plate,
        "base_swivel": base_swivel,
        "elbow_pitch": elbow_pitch,
        "wrist_tilt": wrist_tilt,
        "vesa_rotation": vesa_rotation,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="triple_monitor_arm",
        meta={
            "reference_note": (
                "Triple-arm variant: three identical articulated monitor arms "
                "on a shared clamp-mounted vertical pole; no classification "
                "mismatch suspected."
            )
        },
    )

    black = model.material("satin_black", rgba=(0.015, 0.017, 0.016, 1.0))
    dark = model.material("dark_hardware", rgba=(0.05, 0.055, 0.052, 1.0))
    silver = model.material("brushed_silver", rgba=(0.58, 0.64, 0.62, 1.0))
    arm_gray = model.material("warm_gray_powdercoat", rgba=(0.56, 0.61, 0.58, 1.0))
    materials = (black, dark, silver, arm_gray)

    # ── Root: clamp + vertical pole ──────────────────────────────────────
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
        origin=Origin(xyz=(0.0, 0.0, 0.558)),
        material=dark,
        name="thrust_collar",
    )

    # ── N = 3 arm chains on the shared pole ──────────────────────────────
    for idx in range(NUM_ARMS):
        _build_arm_chain(
            model,
            idx,
            pole_z=ARM_POLE_Z[idx],
            yaw_offset=ARM_YAW[idx],
            clamp_base=clamp_base,
            materials=materials,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")

    # Gather indexed arm references
    arms = []
    for idx in range(NUM_ARMS):
        s = f"_{idx}"
        arms.append({
            "idx": idx,
            "lower_arm": object_model.get_part(f"lower_arm{s}"),
            "upper_arm": object_model.get_part(f"upper_arm{s}"),
            "wrist_head": object_model.get_part(f"wrist_head{s}"),
            "vesa_plate": object_model.get_part(f"vesa_plate{s}"),
            "base_swivel": object_model.get_articulation(f"base_swivel{s}"),
            "elbow_pitch": object_model.get_articulation(f"elbow_pitch{s}"),
            "wrist_tilt": object_model.get_articulation(f"wrist_tilt{s}"),
            "vesa_rotation": object_model.get_articulation(f"vesa_rotation{s}"),
        })

    # ── TARGET: three arm chains share the vertical pole ────────────────
    ctx.check(
        "three arm chains emitted on shared pole",
        len(arms) == NUM_ARMS,
        details=f"expected {NUM_ARMS} arms, found {len(arms)}",
    )

    # Per-arm geometry retention checks
    for arm in arms:
        i = arm["idx"]
        # Swivel sleeve is a zero-clearance bearing on the pole
        ctx.allow_overlap(
            clamp_base,
            arm["lower_arm"],
            elem_a="vertical_pole",
            elem_b="base_swivel_sleeve",
            reason=(
                f"Arm {i}: the swivel sleeve bore is a zero-clearance bearing "
                "fit around the vertical pole for the base swivel joint."
            ),
        )
        ctx.expect_overlap(
            clamp_base,
            arm["lower_arm"],
            axes="xy",
            elem_a="vertical_pole",
            elem_b="base_swivel_sleeve",
            min_overlap=0.030,
            name=f"arm_{i} swivel collar surrounds pole",
        )
        # Elbow clevis wraps around the upper-arm hinge end
        ctx.allow_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            elem_a="elbow_cheek_0",
            elem_b="upper_arm_shell",
            reason=(
                f"Arm {i}: the elbow clevis cheek intentionally wraps around the "
                "upper-arm shell hinge end as a forked hinge fit."
            ),
        )
        ctx.allow_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            elem_a="elbow_cheek_1",
            elem_b="upper_arm_shell",
            reason=(
                f"Arm {i}: the elbow clevis cheek intentionally wraps around the "
                "upper-arm shell hinge end as a forked hinge fit."
            ),
        )
        ctx.expect_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            axes="yz",
            elem_a="elbow_pin",
            elem_b="elbow_eye",
            min_overlap=0.015,
            name=f"arm_{i} elbow eye retained on pin",
        )
        ctx.allow_overlap(
            arm["upper_arm"],
            arm["wrist_head"],
            elem_a="wrist_pin",
            elem_b="wrist_tilt_eye",
            reason=(
                f"Arm {i}: wrist pin is intentionally captured through the "
                "tilt eye bore as a thin captured-pin fit."
            ),
        )
        ctx.expect_overlap(
            arm["upper_arm"],
            arm["wrist_head"],
            axes="yz",
            elem_a="wrist_pin",
            elem_b="wrist_tilt_eye",
            min_overlap=0.012,
            name=f"arm_{i} wrist eye retained on pin",
        )
        ctx.allow_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            elem_a="rotation_bearing",
            elem_b="vesa_backplate",
            reason=(
                f"Arm {i}: rotation bearing is intentionally seated into the "
                "rear of the thin stamped VESA plate."
            ),
        )
        ctx.allow_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            elem_a="rotation_bearing",
            elem_b="center_boss",
            reason=(
                f"Arm {i}: rotation bearing and center boss mate as the "
                "VESA rotation bearing interface."
            ),
        )
        ctx.expect_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            axes="yz",
            elem_a="rotation_bearing",
            elem_b="vesa_backplate",
            min_overlap=0.045,
            name=f"arm_{i} VESA bearing centered behind plate",
        )

    # ── Distinct pole heights (multiplicity spacing) ────────────────────
    sleeve_z = []
    for arm in arms:
        pos = ctx.part_world_position(arm["lower_arm"])
        if pos is not None:
            sleeve_z.append(pos[2])
    if len(sleeve_z) == NUM_ARMS:
        z_sorted = sorted(sleeve_z)
        min_gap = min(z_sorted[i + 1] - z_sorted[i] for i in range(len(z_sorted) - 1))
        ctx.check(
            "arms mounted at distinct pole heights",
            min_gap > 0.08,
            details=f"sleeve_z={sleeve_z}, min_gap={min_gap:.3f} m",
        )

    # ── Articulation proof (arm 0 as representative) ────────────────────
    a0 = arms[0]

    lower_rest_aabb = ctx.part_element_world_aabb(a0["lower_arm"], elem="elbow_pin")
    with ctx.pose({a0["base_swivel"]: 0.75}):
        lower_swiveled_aabb = ctx.part_element_world_aabb(a0["lower_arm"], elem="elbow_pin")
    if lower_rest_aabb is not None and lower_swiveled_aabb is not None:
        rest_y = (lower_rest_aabb[0][1] + lower_rest_aabb[1][1]) / 2.0
        swiveled_y = (lower_swiveled_aabb[0][1] + lower_swiveled_aabb[1][1]) / 2.0
        ctx.check(
            "arm_0 base_swivel swings arm about pole",
            abs(swiveled_y - rest_y) > 0.20,
            details=f"rest_y={rest_y:.3f}, swiveled_y={swiveled_y:.3f}",
        )

    with ctx.pose({a0["elbow_pitch"]: -0.45}):
        wrist_low = ctx.part_world_position(a0["wrist_head"])
    with ctx.pose({a0["elbow_pitch"]: 0.45}):
        wrist_high = ctx.part_world_position(a0["wrist_head"])
    ctx.check(
        "arm_0 elbow_pitch changes wrist height",
        wrist_low is not None
        and wrist_high is not None
        and abs(wrist_high[2] - wrist_low[2]) > 0.12,
        details=f"low={wrist_low}, high={wrist_high}",
    )

    with ctx.pose({a0["wrist_tilt"]: -0.45}):
        plate_low = ctx.part_world_position(a0["vesa_plate"])
    with ctx.pose({a0["wrist_tilt"]: 0.45}):
        plate_high = ctx.part_world_position(a0["vesa_plate"])
    ctx.check(
        "arm_0 wrist_tilt nods VESA plate",
        plate_low is not None
        and plate_high is not None
        and abs(plate_high[2] - plate_low[2]) > 0.060,
        details=f"low={plate_low}, high={plate_high}",
    )

    plate_rest_aabb = ctx.part_world_aabb(a0["vesa_plate"])
    with ctx.pose({a0["vesa_rotation"]: 0.55}):
        plate_rotated_aabb = ctx.part_world_aabb(a0["vesa_plate"])
    if plate_rest_aabb is not None and plate_rotated_aabb is not None:
        rest_h = plate_rest_aabb[1][2] - plate_rest_aabb[0][2]
        rotated_h = plate_rotated_aabb[1][2] - plate_rotated_aabb[0][2]
        ctx.check(
            "arm_0 vesa_rotation rolls mounting plate",
            abs(rotated_h - rest_h) > 0.020,
            details=f"rest_height={rest_h:.3f}, rotated_height={rotated_h:.3f}",
        )

    # ── Independent swivel per arm ───────────────────────────────────────
    for arm in arms:
        i = arm["idx"]
        rest = ctx.part_world_position(arm["vesa_plate"])
        with ctx.pose({arm["base_swivel"]: 0.50}):
            moved = ctx.part_world_position(arm["vesa_plate"])
        if rest is not None and moved is not None:
            ctx.check(
                f"arm_{i} swivels independently on shared pole",
                abs(moved[0] - rest[0]) > 0.05 or abs(moved[1] - rest[1]) > 0.05,
                details=f"rest={rest}, moved={moved}",
            )

    return ctx.report()


object_model = build_object_model()
