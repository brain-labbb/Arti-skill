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


def _build_arm_chain(
    model: ArticulatedObject,
    clamp_base,
    idx: int,
    pole_z: float,
    yaw_offset: float,
    black,
    dark,
    silver,
    arm_gray,
) -> dict:
    """Build one complete arm chain on the shared pole.

    Creates lower_arm -> upper_arm -> wrist_head -> vesa_plate with four
    revolute joints: base_swivel, elbow_pitch, wrist_tilt, vesa_rotation.
    All part and visual names are prefixed with ``arm{idx}_`` for stable
    indexed naming across the two arms.
    """
    p = f"arm{idx}_"

    # ── Lower arm ─────────────────────────────────────────────────────────
    lower_arm = model.part(f"{p}lower_arm")
    lower_arm.visual(
        _tube_mesh(0.036, 0.020, 0.092, f"{p}base_swivel_sleeve"),
        material=silver,
        name=f"{p}base_swivel_sleeve",
    )
    lower_arm.visual(
        _rounded_box_mesh(0.335, 0.052, 0.044, 0.010, f"{p}lower_arm_shell"),
        origin=Origin(xyz=(0.198, 0.0, 0.0)),
        material=arm_gray,
        name=f"{p}lower_arm_shell",
    )
    lower_arm.visual(
        Box((0.026, 0.070, 0.008)),
        origin=Origin(xyz=(0.235, 0.0, -0.024)),
        material=dark,
        name=f"{p}lower_cable_clip",
    )
    lower_arm.visual(
        Box((0.080, 0.015, 0.076)),
        origin=Origin(xyz=(0.400, 0.032, 0.0)),
        material=dark,
        name=f"{p}elbow_cheek_0",
    )
    lower_arm.visual(
        Box((0.080, 0.015, 0.076)),
        origin=Origin(xyz=(0.400, -0.032, 0.0)),
        material=dark,
        name=f"{p}elbow_cheek_1",
    )
    lower_arm.visual(
        Cylinder(radius=0.010, length=0.095),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"{p}elbow_pin",
    )
    lower_arm.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.400, 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name=f"{p}elbow_bolt_head_0",
    )
    lower_arm.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.400, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name=f"{p}elbow_bolt_head_1",
    )

    # ── Upper arm ─────────────────────────────────────────────────────────
    upper_arm = model.part(f"{p}upper_arm")
    upper_arm.visual(
        _tube_mesh(0.027, 0.012, 0.035, f"{p}elbow_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"{p}elbow_eye",
    )
    upper_arm.visual(
        _rounded_box_mesh(0.370, 0.054, 0.052, 0.012, f"{p}upper_arm_shell"),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=arm_gray,
        name=f"{p}upper_arm_shell",
    )
    upper_arm.visual(
        Box((0.310, 0.018, 0.018)),
        origin=Origin(xyz=(0.220, 0.0, -0.047)),
        material=black,
        name=f"{p}gas_spring_link",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.080, 0.0, -0.032)),
        material=black,
        name=f"{p}gas_link_mount_0",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.036)),
        origin=Origin(xyz=(0.350, 0.0, -0.032)),
        material=black,
        name=f"{p}gas_link_mount_1",
    )
    upper_arm.visual(
        Box((0.028, 0.070, 0.008)),
        origin=Origin(xyz=(0.280, 0.0, -0.030)),
        material=dark,
        name=f"{p}upper_cable_clip",
    )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.420, 0.028, 0.0)),
        material=dark,
        name=f"{p}wrist_cheek_0",
    )
    upper_arm.visual(
        Box((0.064, 0.012, 0.060)),
        origin=Origin(xyz=(0.420, -0.028, 0.0)),
        material=dark,
        name=f"{p}wrist_cheek_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.008, length=0.078),
        origin=Origin(xyz=(0.420, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"{p}wrist_pin",
    )
    upper_arm.visual(
        Box((0.070, 0.070, 0.010)),
        origin=Origin(xyz=(0.410, 0.0, -0.029)),
        material=dark,
        name=f"{p}wrist_yoke_bridge",
    )

    # ── Wrist head ────────────────────────────────────────────────────────
    wrist_head = model.part(f"{p}wrist_head")
    wrist_head.visual(
        _tube_mesh(0.022, 0.008, 0.028, f"{p}wrist_tilt_eye"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name=f"{p}wrist_tilt_eye",
    )
    wrist_head.visual(
        _rounded_box_mesh(0.070, 0.036, 0.036, 0.007, f"{p}wrist_neck"),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=black,
        name=f"{p}wrist_neck",
    )
    wrist_head.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.103, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name=f"{p}rotation_bearing",
    )

    # ── VESA plate ────────────────────────────────────────────────────────
    vesa_plate = model.part(f"{p}vesa_plate")
    vesa_plate.visual(
        Box((0.010, 0.118, 0.118)),
        material=black,
        name=f"{p}vesa_backplate",
    )
    for vi, (y, z) in enumerate(
        ((0.052, 0.052), (0.052, -0.052), (-0.052, 0.052), (-0.052, -0.052))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"{p}corner_ear_{vi}",
        )
    vesa_plate.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name=f"{p}center_boss",
    )
    for vi, (y, z) in enumerate(
        ((0.0375, 0.0375), (0.0375, -0.0375), (-0.0375, 0.0375), (-0.0375, -0.0375))
    ):
        vesa_plate.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.008, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=silver,
            name=f"{p}vesa_fastener_{vi}",
        )

    # ── Articulations ─────────────────────────────────────────────────────
    base_swivel = model.articulation(
        f"{p}base_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, pole_z), rpy=(0.0, 0.0, yaw_offset)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    elbow_pitch = model.articulation(
        f"{p}elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(0.0, -0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=-0.75, upper=0.85),
    )
    wrist_tilt = model.articulation(
        f"{p}wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist_head,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.85, upper=0.85),
    )
    vesa_rotation = model.articulation(
        f"{p}vesa_rotation",
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
        name="adjustable_monitor_arm",
        meta={
            "reference_note": (
                "The reference reads as a clamp-mounted dual adjustable monitor arm; "
                "no classification mismatch suspected."
            )
        },
    )

    black = model.material("satin_black", rgba=(0.015, 0.017, 0.016, 1.0))
    dark = model.material("dark_hardware", rgba=(0.05, 0.055, 0.052, 1.0))
    silver = model.material("brushed_silver", rgba=(0.58, 0.64, 0.62, 1.0))
    arm_gray = model.material("warm_gray_powdercoat", rgba=(0.56, 0.61, 0.58, 1.0))

    # ── Root: clamp base with extended vertical pole ──────────────────────
    # The pole is extended to 0.600 m to carry two independent arm chains
    # at different heights.
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
    # Extended pole: 0.600 m long, center at z=0.374, spans z=0.074 to z=0.674
    clamp_base.visual(
        Cylinder(radius=0.020, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=silver,
        name="vertical_pole",
    )
    # Top cap collar
    clamp_base.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.667)),
        material=dark,
        name="top_collar",
    )
    # Intermediate stop collar between the two arm sleeves
    clamp_base.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.497)),
        material=dark,
        name="mid_collar",
    )

    # ── Build two arm chains via loop ─────────────────────────────────────
    # Arm 0 (upper): pole_z=0.570, yaw_offset=+0.25 rad (fanned slightly left)
    # Arm 1 (lower): pole_z=0.430, yaw_offset=-0.25 rad (fanned slightly right)
    # The radial fan ensures the two arms rest at distinct azimuthal angles
    # and can swivel/fold/tilt independently without collision at rest.
    arm_configs = [
        {"idx": 0, "pole_z": 0.570, "yaw_offset": 0.25},
        {"idx": 1, "pole_z": 0.430, "yaw_offset": -0.25},
    ]
    arms = []
    for cfg in arm_configs:
        arm = _build_arm_chain(
            model,
            clamp_base,
            idx=cfg["idx"],
            pole_z=cfg["pole_z"],
            yaw_offset=cfg["yaw_offset"],
            black=black,
            dark=dark,
            silver=silver,
            arm_gray=arm_gray,
        )
        arms.append(arm)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")

    # Collect both arm chains
    arms = []
    for idx in range(2):
        p = f"arm{idx}_"
        arms.append({
            "lower_arm": object_model.get_part(f"{p}lower_arm"),
            "upper_arm": object_model.get_part(f"{p}upper_arm"),
            "wrist_head": object_model.get_part(f"{p}wrist_head"),
            "vesa_plate": object_model.get_part(f"{p}vesa_plate"),
            "base_swivel": object_model.get_articulation(f"{p}base_swivel"),
            "elbow_pitch": object_model.get_articulation(f"{p}elbow_pitch"),
            "wrist_tilt": object_model.get_articulation(f"{p}wrist_tilt"),
            "vesa_rotation": object_model.get_articulation(f"{p}vesa_rotation"),
        })

    ctx.check(
        "reference classification",
        object_model.meta.get("reference_note", "").startswith("The reference reads as"),
        details=str(object_model.meta.get("reference_note")),
    )

    # ── Structural: both arms exist on the shared pole ────────────────────
    ctx.check(
        "dual arm count: two independent arm chains on shared pole",
        len(arms) == 2,
        details=f"expected 2 arm chains, got {len(arms)}",
    )

    for idx, arm in enumerate(arms):
        p = f"arm{idx}_"

        # Swivel collar surrounds vertical pole in plan
        ctx.allow_overlap(
            clamp_base,
            arm["lower_arm"],
            elem_a="vertical_pole",
            elem_b=f"{p}base_swivel_sleeve",
            reason=(
                f"The arm{idx} swivel sleeve is intentionally wrapped around the "
                "shared vertical pole as the base swivel bearing interface."
            ),
        )
        ctx.expect_overlap(
            clamp_base,
            arm["lower_arm"],
            axes="xy",
            elem_a="vertical_pole",
            elem_b=f"{p}base_swivel_sleeve",
            min_overlap=0.030,
            name=f"arm{idx} swivel collar surrounds vertical pole in plan",
        )
        ctx.expect_within(
            arm["lower_arm"],
            clamp_base,
            axes="xy",
            inner_elem=f"{p}base_swivel_sleeve",
            outer_elem="vertical_pole",
            margin=0.020,
            name=f"arm{idx} swivel sleeve is concentric with the pole",
        )

        # Elbow eye retained on hinge pin
        ctx.expect_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            axes="yz",
            elem_a=f"{p}elbow_pin",
            elem_b=f"{p}elbow_eye",
            min_overlap=0.015,
            name=f"arm{idx} elbow eye is retained on the hinge pin",
        )

        # Elbow clevis: upper arm shell nests between the two cheeks
        ctx.allow_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            elem_a=f"{p}elbow_cheek_0",
            elem_b=f"{p}upper_arm_shell",
            reason=(
                f"The arm{idx} upper arm shell is intentionally seated between the "
                "elbow clevis cheeks as the hinge joint interface."
            ),
        )
        ctx.allow_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            elem_a=f"{p}elbow_cheek_1",
            elem_b=f"{p}upper_arm_shell",
            reason=(
                f"The arm{idx} upper arm shell is intentionally seated between the "
                "elbow clevis cheeks as the hinge joint interface."
            ),
        )
        ctx.expect_overlap(
            arm["lower_arm"],
            arm["upper_arm"],
            axes="xy",
            elem_a=f"{p}elbow_pin",
            elem_b=f"{p}elbow_eye",
            min_overlap=0.010,
            name=f"arm{idx} elbow pin spans the clevis joint",
        )

        # Wrist pin captured through tilt eye
        ctx.allow_overlap(
            arm["upper_arm"],
            arm["wrist_head"],
            elem_a=f"{p}wrist_pin",
            elem_b=f"{p}wrist_tilt_eye",
            reason=(
                f"The arm{idx} wrist pin is intentionally captured through the tilt eye; "
                "the thin hinge bore is represented by coincident pin-and-eye geometry."
            ),
        )
        ctx.expect_overlap(
            arm["upper_arm"],
            arm["wrist_head"],
            axes="yz",
            elem_a=f"{p}wrist_pin",
            elem_b=f"{p}wrist_tilt_eye",
            min_overlap=0.012,
            name=f"arm{idx} wrist tilt eye is retained on the wrist pin",
        )

        # VESA rotation bearing seated into plate
        ctx.allow_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            elem_a=f"{p}rotation_bearing",
            elem_b=f"{p}vesa_backplate",
            reason=(
                f"The arm{idx} wrist rotation bearing is intentionally seated into the rear "
                "of the thin stamped VESA plate as a local captured mounting boss."
            ),
        )
        ctx.expect_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            axes="yz",
            elem_a=f"{p}rotation_bearing",
            elem_b=f"{p}vesa_backplate",
            min_overlap=0.045,
            name=f"arm{idx} VESA rotation bearing is centered behind the plate",
        )
        ctx.allow_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            elem_a=f"{p}rotation_bearing",
            elem_b=f"{p}center_boss",
            reason=(
                f"The arm{idx} rotation bearing is intentionally seated into the "
                "VESA plate center boss as the rotation pivot mounting."
            ),
        )
        ctx.expect_overlap(
            arm["wrist_head"],
            arm["vesa_plate"],
            axes="yz",
            elem_a=f"{p}rotation_bearing",
            elem_b=f"{p}center_boss",
            min_overlap=0.020,
            name=f"arm{idx} rotation bearing is retained in center boss",
        )

    # ── Articulation behavior: each arm moves independently ────────────────
    for idx, arm in enumerate(arms):
        base_swivel = arm["base_swivel"]
        elbow_pitch = arm["elbow_pitch"]
        wrist_tilt = arm["wrist_tilt"]
        vesa_rotation = arm["vesa_rotation"]

        # Base swivel swings the arm about the pole
        lower_rest_aabb = ctx.part_element_world_aabb(
            arm["lower_arm"], elem=f"arm{idx}_elbow_pin"
        )
        with ctx.pose({base_swivel: 0.75}):
            lower_swiveled_aabb = ctx.part_element_world_aabb(
                arm["lower_arm"], elem=f"arm{idx}_elbow_pin"
            )
        if lower_rest_aabb is not None and lower_swiveled_aabb is not None:
            rest_y = (lower_rest_aabb[0][1] + lower_rest_aabb[1][1]) / 2.0
            swiveled_y = (lower_swiveled_aabb[0][1] + lower_swiveled_aabb[1][1]) / 2.0
            ctx.check(
                f"arm{idx} base swivel swings arm about pole",
                abs(swiveled_y - rest_y) > 0.20,
                details=f"rest_y={rest_y:.3f}, swiveled_y={swiveled_y:.3f}",
            )

        # Elbow pitch changes wrist height
        with ctx.pose({elbow_pitch: -0.45}):
            wrist_low = ctx.part_world_position(arm["wrist_head"])
        with ctx.pose({elbow_pitch: 0.45}):
            wrist_high = ctx.part_world_position(arm["wrist_head"])
        ctx.check(
            f"arm{idx} elbow pitch changes wrist height",
            wrist_low is not None
            and wrist_high is not None
            and abs(wrist_high[2] - wrist_low[2]) > 0.12,
            details=f"low={wrist_low}, high={wrist_high}",
        )

        # Wrist tilt nods the VESA plate
        with ctx.pose({wrist_tilt: -0.45}):
            plate_low = ctx.part_world_position(arm["vesa_plate"])
        with ctx.pose({wrist_tilt: 0.45}):
            plate_high = ctx.part_world_position(arm["vesa_plate"])
        ctx.check(
            f"arm{idx} wrist tilt nods the VESA plate",
            plate_low is not None
            and plate_high is not None
            and abs(plate_high[2] - plate_low[2]) > 0.060,
            details=f"low={plate_low}, high={plate_high}",
        )

        # VESA rotation rolls the mounting plate
        plate_rest_aabb = ctx.part_world_aabb(arm["vesa_plate"])
        with ctx.pose({vesa_rotation: 0.55}):
            plate_rotated_aabb = ctx.part_world_aabb(arm["vesa_plate"])
        if plate_rest_aabb is not None and plate_rotated_aabb is not None:
            rest_height = plate_rest_aabb[1][2] - plate_rest_aabb[0][2]
            rotated_height = plate_rotated_aabb[1][2] - plate_rotated_aabb[0][2]
            ctx.check(
                f"arm{idx} VESA rotation rolls the mounting plate",
                abs(rotated_height - rest_height) > 0.020,
                details=f"rest_height={rest_height:.3f}, rotated_height={rotated_height:.3f}",
            )

    # ── Radial fan: the two arms rest at distinct azimuthal positions ─────
    # Check the elbow pin (far end of each lower arm) to measure the lateral
    # separation created by the distinct yaw offsets.
    arm0_tip = ctx.part_element_world_aabb(arms[0]["lower_arm"], elem="arm0_elbow_pin")
    arm1_tip = ctx.part_element_world_aabb(arms[1]["lower_arm"], elem="arm1_elbow_pin")
    if arm0_tip is not None and arm1_tip is not None:
        arm0_cx = (arm0_tip[0][0] + arm0_tip[1][0]) / 2.0
        arm0_cy = (arm0_tip[0][1] + arm0_tip[1][1]) / 2.0
        arm1_cx = (arm1_tip[0][0] + arm1_tip[1][0]) / 2.0
        arm1_cy = (arm1_tip[0][1] + arm1_tip[1][1]) / 2.0
        lateral_separation = math.sqrt(
            (arm0_cx - arm1_cx) ** 2 + (arm0_cy - arm1_cy) ** 2
        )
        ctx.check(
            "dual arm radial fan: arms rest at distinct azimuthal positions",
            lateral_separation > 0.05,
            details=(
                f"arm0_elbow_pin=({arm0_cx:.3f},{arm0_cy:.3f}), "
                f"arm1_elbow_pin=({arm1_cx:.3f},{arm1_cy:.3f}), "
                f"lateral_sep={lateral_separation:.4f}"
            ),
        )

    # ── Vertical separation: arms are at distinct pole heights ────────────
    arm0_z = ctx.part_world_position(arms[0]["lower_arm"])
    arm1_z = ctx.part_world_position(arms[1]["lower_arm"])
    if arm0_z is not None and arm1_z is not None:
        height_diff = abs(arm0_z[2] - arm1_z[2])
        ctx.check(
            "dual arm vertical separation on shared pole",
            height_diff > 0.10,
            details=f"arm0_z={arm0_z[2]:.3f}, arm1_z={arm1_z[2]:.3f}, diff={height_diff:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
