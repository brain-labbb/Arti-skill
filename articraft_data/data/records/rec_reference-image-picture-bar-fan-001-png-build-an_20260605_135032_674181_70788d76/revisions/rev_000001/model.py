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


MOTOR_Z = 0.38
BLADE_ROOT_R = 0.13
BLADE_LENGTH = 0.56
BLADE_WIDTH = 0.070
BLADE_T = 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ornate_ceiling_fan")

    dark_bronze = model.material("dark_bronze", rgba=(0.075, 0.060, 0.048, 1.0))
    antique_gold = model.material("antique_gold", rgba=(0.74, 0.55, 0.30, 1.0))
    black = model.material("black_shadow", rgba=(0.018, 0.016, 0.014, 1.0))
    wood = model.material("warm_wood", rgba=(0.72, 0.38, 0.15, 1.0))
    wood_dark = model.material("wood_edge", rgba=(0.42, 0.20, 0.08, 1.0))

    mount = model.part("ceiling_mount")
    mount.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=dark_bronze,
        name="ceiling_canopy_outer",
    )
    mount.visual(
        Sphere(radius=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.870), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="rounded_canopy_bowl",
    )
    mount.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material=dark_bronze,
        name="ribbed_downrod_core",
    )
    for i, z in enumerate((0.43, 0.49, 0.55, 0.61, 0.67, 0.73, 0.79)):
        mount.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=antique_gold if i % 2 else black,
            name=f"downrod_ring_{i}",
        )
    mount.visual(
        Cylinder(radius=0.125, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, MOTOR_Z + 0.035)),
        material=dark_bronze,
        name="upper_motor_bell",
    )
    mount.visual(
        Cylinder(radius=0.136, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, MOTOR_Z + 0.083)),
        material=antique_gold,
        name="upper_ornate_rim",
    )
    mount.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, MOTOR_Z - 0.003)),
        material=antique_gold,
        name="lower_static_trim",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.088, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=black,
        name="lower_motor_hub",
    )
    rotor.visual(
        Cylinder(radius=0.112, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=antique_gold,
        name="spoked_gold_ring",
    )
    rotor.visual(
        Sphere(radius=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        material=dark_bronze,
        name="center_spinner",
    )
    rotor.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.108)),
        material=antique_gold,
        name="bottom_finial",
    )

    for i in range(4):
        angle = i * math.pi / 2.0 + math.radians(10.0)
        c = math.cos(angle)
        s = math.sin(angle)
        root_x = c * 0.088
        root_y = s * 0.088
        bracket_mid_r = 0.118
        blade_mid_r = BLADE_ROOT_R + BLADE_LENGTH / 2.0
        tip_r = BLADE_ROOT_R + BLADE_LENGTH - 0.030
        yaw = angle

        rotor.visual(
            Box((0.105, 0.018, 0.014)),
            origin=Origin(
                xyz=(c * bracket_mid_r, s * bracket_mid_r, -0.020),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_bronze,
            name=f"blade_bracket_{i}",
        )
        rotor.visual(
            Box((0.110, BLADE_WIDTH * 0.72, 0.014)),
            origin=Origin(
                xyz=(c * (BLADE_ROOT_R + 0.040), s * (BLADE_ROOT_R + 0.040), -0.026),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark_bronze,
            name=f"blade_root_clamp_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(
                xyz=(root_x, root_y, -0.020),
                rpy=(math.pi / 2.0, 0.0, yaw),
            ),
            material=antique_gold,
            name=f"root_boss_{i}",
        )
        rotor.visual(
            Box((BLADE_LENGTH, BLADE_WIDTH, BLADE_T)),
            origin=Origin(
                xyz=(c * blade_mid_r, s * blade_mid_r, -0.026),
                rpy=(0.0, 0.0, yaw),
            ),
            material=wood,
            name=f"wood_blade_{i}",
        )
        rotor.visual(
            Box((0.035, BLADE_WIDTH * 0.92, BLADE_T + 0.001)),
            origin=Origin(
                xyz=(c * tip_r, s * tip_r, -0.026),
                rpy=(0.0, 0.0, yaw),
            ),
            material=wood_dark,
            name=f"rounded_blade_tip_{i}",
        )
        rotor.visual(
            Box((0.018, 0.018, 0.006)),
            origin=Origin(
                xyz=(c * 0.165 - s * 0.005, s * 0.165 + c * 0.005, -0.021),
                rpy=(0.0, 0.0, yaw),
            ),
            material=antique_gold,
            name=f"bracket_screw_0_{i}",
        )
        rotor.visual(
            Box((0.018, 0.018, 0.006)),
            origin=Origin(
                xyz=(c * 0.165 + s * 0.005, s * 0.165 - c * 0.005, -0.021),
                rpy=(0.0, 0.0, yaw),
            ),
            material=antique_gold,
            name=f"bracket_screw_1_{i}",
        )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, MOTOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("ceiling_mount")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("mount_to_rotor")

    ctx.allow_overlap(
        mount,
        rotor,
        elem_a="lower_static_trim",
        elem_b="spoked_gold_ring",
        reason="The rotating decorative ring nests slightly under the stationary motor trim.",
    )

    ctx.expect_contact(
        mount,
        rotor,
        elem_a="lower_static_trim",
        elem_b="spoked_gold_ring",
        contact_tol=0.004,
        name="rotor is seated under the motor housing",
    )
    ctx.expect_overlap(
        rotor,
        mount,
        axes="xy",
        min_overlap=0.08,
        elem_a="lower_motor_hub",
        elem_b="upper_motor_bell",
        name="rotor hub stays centered on the motor",
    )

    blade_visuals = [visual for visual in rotor.visuals if visual.name and visual.name.startswith("wood_blade_")]
    bracket_visuals = [
        visual for visual in rotor.visuals if visual.name and visual.name.startswith("blade_bracket_")
    ]
    ctx.check(
        "fan has four wood blades",
        len(blade_visuals) == 4,
        details=f"blade_count={len(blade_visuals)}",
    )
    ctx.check(
        "each blade has a bronze root bracket",
        len(bracket_visuals) == 4,
        details=f"bracket_count={len(bracket_visuals)}",
    )
    ctx.check(
        "primary articulation is continuous spin",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    rest = ctx.part_element_world_aabb(rotor, elem="wood_blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        turned = ctx.part_element_world_aabb(rotor, elem="wood_blade_0")
    ctx.check(
        "blade rotates around the vertical motor axis",
        rest is not None
        and turned is not None
        and abs((rest[1][0] - rest[0][0]) - (turned[1][1] - turned[0][1])) < 0.08,
        details=f"rest={rest}, turned={turned}",
    )

    return ctx.report()


object_model = build_object_model()
