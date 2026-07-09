from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


TIRE_RADIUS = 0.145
TIRE_WIDTH = 0.064
WHEEL_CENTER_Z = -0.240


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_landing_gear")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.016, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.50, 0.52, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.15, 0.16, 1.0))
    brass_hub = model.material("brass_hub", rgba=(0.72, 0.52, 0.28, 1.0))
    bolt_silver = model.material("bolt_silver", rgba=(0.86, 0.86, 0.82, 1.0))

    mount = model.part("mount_plate")
    mount.visual(
        Box((0.240, 0.026, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=matte_black,
        name="backing_plate",
    )
    # Four visible fasteners in the compact black plate, matching the reference.
    for i, (x, z) in enumerate(((-0.095, 0.190), (0.095, 0.190), (-0.095, 0.035), (0.095, 0.035))):
        mount.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(xyz=(x, -0.016, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=bolt_silver,
            name=f"plate_bolt_{i}",
        )
        mount.visual(
            Cylinder(radius=0.0045, length=0.008),
            origin=Origin(xyz=(x, -0.020, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"bolt_socket_{i}",
        )

    # Retraction hinge lugs stand proud of the plate and leave a central clevis gap.
    for side, x, pad_x in (("0", -0.090, -0.122), ("1", 0.090, 0.122)):
        mount.visual(
            Box((0.030, 0.045, 0.070)),
            origin=Origin(xyz=(x, -0.035, -0.004)),
            material=dark_steel,
            name=f"hinge_lug_{side}",
        )
        mount.visual(
            Cylinder(radius=0.013, length=0.006),
            origin=Origin(xyz=(x, -0.058, -0.004), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name=f"lug_bushing_{side}",
        )
        mount.visual(
            Box((0.026, 0.020, 0.028)),
            origin=Origin(xyz=(pad_x, -0.020, 0.080)),
            material=dark_steel,
            name=f"brace_pad_{side}",
        )

    main_strut = model.part("main_strut")
    main_strut.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="pivot_barrel",
    )
    main_strut.visual(
        Box((0.052, 0.038, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=brushed_steel,
        name="trunnion_block",
    )
    main_strut.visual(
        Cylinder(radius=0.030, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, -0.178)),
        material=brushed_steel,
        name="outer_sleeve",
    )
    main_strut.visual(
        Cylinder(radius=0.020, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.342)),
        material=dark_steel,
        name="gland_collar",
    )
    slider = model.part("shock_slider")
    slider.visual(
        Cylinder(radius=0.0135, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=polished_steel,
        name="polished_rod",
    )
    slider.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.306)),
        material=brushed_steel,
        name="lower_collar",
    )
    slider.visual(
        Cylinder(radius=0.017, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        material=dark_steel,
        name="caster_socket",
    )

    fork = model.part("caster_fork")
    fork.visual(
        Cylinder(radius=0.014, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=polished_steel,
        name="steering_stem",
    )
    fork.visual(
        Box((0.140, 0.042, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=brushed_steel,
        name="fork_crown",
    )
    fork.visual(
        Box((0.019, 0.034, 0.218)),
        origin=Origin(xyz=(-0.057, 0.0, -0.198)),
        material=brushed_steel,
        name="fork_blade_0",
    )
    fork.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(-0.057, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="axle_boss_0",
    )
    fork.visual(
        Cylinder(radius=0.008, length=0.007),
        origin=Origin(xyz=(-0.066, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bolt_silver,
        name="axle_nut_0",
    )
    fork.visual(
        Box((0.019, 0.034, 0.218)),
        origin=Origin(xyz=(0.057, 0.0, -0.198)),
        material=brushed_steel,
        name="fork_blade_1",
    )
    fork.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.057, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=polished_steel,
        name="axle_boss_1",
    )
    fork.visual(
        Cylinder(radius=0.008, length=0.007),
        origin=Origin(xyz=(0.066, 0.0, WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bolt_silver,
        name="axle_nut_1",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.100,
                tread=TireTread(style="circumferential", depth=0.0045, count=4),
                grooves=(
                    TireGroove(center_offset=-0.015, width=0.004, depth=0.0025),
                    TireGroove(center_offset=0.015, width=0.004, depth=0.0025),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.055),
                shoulder=TireShoulder(width=0.006, radius=0.004),
            ),
            "landing_gear_tire",
        ),
        material=dark_rubber,
        name="tire",
    )
    wheel.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.104,
                0.054,
                rim=WheelRim(inner_radius=0.064, flange_height=0.009, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.031,
                    width=0.050,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.047, hole_diameter=0.005),
                ),
                face=WheelFace(dish_depth=0.007, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.006, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.017),
            ),
            "landing_gear_wheel_hub",
        ),
        material=brass_hub,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bolt_silver,
        name="center_cap",
    )
    for i, (y, z) in enumerate(
        (
            (0.000, 0.020),
            (0.019, 0.006),
            (0.012, -0.018),
            (-0.012, -0.018),
            (-0.019, 0.006),
        )
    ):
        wheel.visual(
            Cylinder(radius=0.0045, length=0.014),
            origin=Origin(xyz=(0.030, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_silver,
            name=f"hub_nut_{i}",
        )

    for index, sign in enumerate((-1.0, 1.0)):
        brace = model.part(f"side_brace_{index}")
        brace.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.0, 0.0, 0.0),
                        (-sign * 0.004, 0.008, -0.150),
                        (-sign * 0.030, 0.012, -0.300),
                        (-sign * 0.052, 0.010, -0.452),
                    ],
                    radius=0.0065,
                    samples_per_segment=8,
                    radial_segments=14,
                ),
                f"side_brace_{index}_rod",
            ),
            material=polished_steel,
            name="brace_rod",
        )
        brace.visual(
            Cylinder(radius=0.014, length=0.022),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name="top_eye",
        )
        brace.visual(
            Cylinder(radius=0.014, length=0.022),
            origin=Origin(xyz=(-sign * 0.052, 0.010, -0.452), rpy=(0.0, pi / 2.0, 0.0)),
            material=brushed_steel,
            name="lower_eye",
        )

    model.articulation(
        "mount_to_strut",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=main_strut,
        origin=Origin(xyz=(0.0, -0.035, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "strut_to_slider",
        ArticulationType.PRISMATIC,
        parent=main_strut,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.045),
    )
    model.articulation(
        "slider_to_fork",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    for index, sign in enumerate((-1.0, 1.0)):
        model.articulation(
            f"mount_to_brace_{index}",
            ArticulationType.REVOLUTE,
            parent=mount,
            child=f"side_brace_{index}",
            origin=Origin(xyz=(sign * 0.122, -0.044, 0.080)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.70, upper=0.15),
            mimic=Mimic("mount_to_strut", multiplier=-0.48, offset=0.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.warn(
        "Classification note: the visual reference reads as an aircraft/robot retractable landing gear; "
        "the Astronomy folder context appears mismatched."
    )

    mount = object_model.get_part("mount_plate")
    strut = object_model.get_part("main_strut")
    slider = object_model.get_part("shock_slider")
    fork = object_model.get_part("caster_fork")
    wheel = object_model.get_part("wheel")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_slider")
    caster = object_model.get_articulation("slider_to_fork")

    ctx.allow_overlap(
        strut,
        slider,
        elem_a="outer_sleeve",
        elem_b="polished_rod",
        reason="The polished oleo rod is intentionally captured slightly inside the upper shock sleeve.",
    )
    ctx.allow_overlap(
        strut,
        slider,
        elem_a="gland_collar",
        elem_b="polished_rod",
        reason="The solid collar is a simplified bearing gland around the telescoping shock rod.",
    )
    ctx.expect_within(
        slider,
        strut,
        axes="xy",
        inner_elem="polished_rod",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="shock rod stays centered in sleeve",
    )
    ctx.expect_overlap(
        strut,
        slider,
        axes="z",
        elem_a="outer_sleeve",
        elem_b="polished_rod",
        min_overlap=0.010,
        name="shock rod remains inserted in sleeve",
    )
    ctx.expect_within(
        slider,
        strut,
        axes="xy",
        inner_elem="polished_rod",
        outer_elem="gland_collar",
        margin=0.001,
        name="shock rod passes through gland collar",
    )
    ctx.expect_overlap(
        strut,
        slider,
        axes="z",
        elem_a="gland_collar",
        elem_b="polished_rod",
        min_overlap=0.030,
        name="gland collar surrounds shock rod",
    )

    ctx.expect_gap(
        fork,
        wheel,
        axis="x",
        positive_elem="fork_blade_1",
        negative_elem="tire",
        min_gap=0.005,
        name="right fork clears tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        fork,
        axis="x",
        positive_elem="tire",
        negative_elem="fork_blade_0",
        min_gap=0.005,
        name="left fork clears tire sidewall",
    )
    ctx.expect_overlap(
        fork,
        wheel,
        axes="z",
        elem_a="axle_boss_0",
        elem_b="hub",
        min_overlap=0.020,
        name="axle bosses align with wheel hub",
    )

    closed_aabb = ctx.part_world_aabb(wheel)
    with ctx.pose({retract: 1.15}):
        retracted_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "retraction swings wheel rearward and upward",
        closed_aabb is not None
        and retracted_aabb is not None
        and retracted_aabb[0][1] > closed_aabb[0][1] + 0.10
        and retracted_aabb[0][2] > closed_aabb[0][2] + 0.10,
        details=f"closed={closed_aabb}, retracted={retracted_aabb}",
    )

    rest_slider_aabb = ctx.part_world_aabb(fork)
    with ctx.pose({shock: 0.040}):
        extended_slider_aabb = ctx.part_world_aabb(fork)
    ctx.check(
        "shock slider extends lower fork",
        rest_slider_aabb is not None
        and extended_slider_aabb is not None
        and extended_slider_aabb[0][2] < rest_slider_aabb[0][2] - 0.030,
        details=f"rest={rest_slider_aabb}, extended={extended_slider_aabb}",
    )

    straight_aabb = ctx.part_world_aabb(wheel)
    with ctx.pose({caster: 0.45}):
        caster_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "caster rotation changes wheel orientation",
        straight_aabb is not None
        and caster_aabb is not None
        and abs((caster_aabb[1][0] - caster_aabb[0][0]) - (straight_aabb[1][0] - straight_aabb[0][0])) > 0.010,
        details=f"straight={straight_aabb}, caster={caster_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
