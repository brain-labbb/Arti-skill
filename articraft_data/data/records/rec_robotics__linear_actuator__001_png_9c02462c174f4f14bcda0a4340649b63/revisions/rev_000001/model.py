from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="compact_linear_actuator_stage",
        meta={
            "category": "Robotics / Linear actuator",
            "reference_note": "Reference image and category both read as a compact rail-style linear actuator.",
        },
    )

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.72, 0.68, 1.0))
    dark_aluminum = model.material("black_anodized", color=(0.015, 0.016, 0.018, 1.0))
    steel = model.material("polished_steel", color=(0.82, 0.84, 0.82, 1.0))
    shadow = model.material("recess_black", color=(0.0, 0.0, 0.0, 1.0))
    screw_steel = model.material("lead_screw_steel", color=(0.64, 0.66, 0.64, 1.0))

    frame = model.part("frame")

    # Long aluminum extrusion and its dark T-slot grooves.
    frame.visual(
        Box((0.700, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=aluminum,
        name="rail_extrusion",
    )
    frame.visual(
        Box((0.680, 0.112, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=dark_aluminum,
        name="bottom_foot",
    )
    for y, name in ((0.044, "side_slot_0"), (-0.044, "side_slot_1")):
        frame.visual(
            Box((0.620, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.041)),
            material=shadow,
            name=name,
        )
    for y, name in ((0.023, "top_slot_0"), (-0.023, "top_slot_1")):
        frame.visual(
            Box((0.610, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.059)),
            material=shadow,
            name=name,
        )

    # Black end plates and left motor housing, as in the reference actuator.
    for x, name in ((-0.370, "end_plate_0"), (0.370, "end_plate_1")):
        frame.visual(
            Box((0.050, 0.118, 0.090)),
            origin=Origin(xyz=(x, 0.0, 0.045)),
            material=dark_aluminum,
            name=name,
        )
    frame.visual(
        Box((0.084, 0.128, 0.095)),
        origin=Origin(xyz=(-0.432, 0.0, 0.061)),
        material=dark_aluminum,
        name="motor_block",
    )
    frame.visual(
        Box((0.090, 0.106, 0.040)),
        origin=Origin(xyz=(-0.445, 0.0, 0.128)),
        material=dark_aluminum,
        name="motor_cap",
    )

    # Parallel guide rods pass through the end plates and frame the carriage path.
    for y, name in ((0.032, "guide_rod_0"), (-0.032, "guide_rod_1")):
        frame.visual(
            Cylinder(radius=0.0065, length=0.735),
            origin=Origin(xyz=(0.0, y, 0.077), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )

    # Small screw-support yokes flank the screw without occupying its clearance.
    for x, side in ((-0.322, "0"), (0.322, "1")):
        for y, suffix in ((0.017, "a"), (-0.017, "b")):
            frame.visual(
                Box((0.014, 0.008, 0.033)),
                origin=Origin(xyz=(x, y, 0.076)),
                material=dark_aluminum,
                name=f"screw_yoke_{side}_{suffix}",
            )
        frame.visual(
            Box((0.014, 0.054, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=dark_aluminum,
            name=f"screw_yoke_base_{side}",
        )

    # Visible screw heads / mounting holes on the black end plates.
    for x, face_name, sign in ((-0.394, "left", -1.0), (0.394, "right", 1.0)):
        for y in (-0.043, 0.043):
            for z in (0.022, 0.068):
                frame.visual(
                    Cylinder(radius=0.0052, length=0.004),
                    origin=Origin(
                        xyz=(x, y, z),
                        rpy=(0.0, math.pi / 2.0, 0.0),
                    ),
                    material=steel,
                    name=f"{face_name}_face_screw_{int((y + 0.05) * 1000)}_{int(z * 1000)}",
                )
    for y in (-0.041, 0.041):
        frame.visual(
            Cylinder(radius=0.0042, length=0.004),
            origin=Origin(xyz=(-0.432, y, 0.108), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"motor_mount_screw_{int((y + 0.05) * 1000)}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.130, 0.090, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=aluminum,
        name="carriage_block",
    )
    carriage.visual(
        Box((0.106, 0.020, 0.015)),
        origin=Origin(xyz=(0.0, 0.037, -0.032)),
        material=steel,
        name="bearing_shoe_0",
    )
    carriage.visual(
        Box((0.106, 0.020, 0.015)),
        origin=Origin(xyz=(0.0, -0.037, -0.032)),
        material=steel,
        name="bearing_shoe_1",
    )
    for y, name in ((0.047, "side_skirt_0"), (-0.047, "side_skirt_1")):
        carriage.visual(
            Box((0.112, 0.010, 0.030)),
            origin=Origin(xyz=(0.0, y, -0.020)),
            material=aluminum,
            name=name,
        )
    carriage.visual(
        Box((0.072, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=aluminum,
        name="raised_mount_boss",
    )
    # Top and side mounting holes are modeled as dark recessed circular faces.
    for x in (-0.042, 0.0, 0.042):
        for y in (-0.022, 0.022):
            carriage.visual(
                Cylinder(radius=0.0054, length=0.004),
                origin=Origin(xyz=(x, y, 0.023), rpy=(0.0, 0.0, 0.0)),
                material=shadow,
                name=f"top_hole_{int((x + 0.05) * 1000)}_{int((y + 0.03) * 1000)}",
            )
    for x in (-0.048, -0.018, 0.018, 0.048):
        carriage.visual(
            Cylinder(radius=0.0043, length=0.004),
            origin=Origin(xyz=(x, -0.047, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name=f"front_hole_{int((x + 0.06) * 1000)}",
        )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        Cylinder(radius=0.0055, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_steel,
        name="screw_core",
    )
    # Closely spaced raised rings give the exposed shaft a recognizable threaded silhouette.
    for i in range(33):
        x = -0.288 + i * 0.018
        lead_screw.visual(
            Cylinder(radius=0.0074, length=0.0032),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_steel,
            name=f"thread_crest_{i:02d}",
        )
    for x, name in ((-0.304, "coupler_0"), (0.304, "coupler_1")):
        lead_screw.visual(
            Cylinder(radius=0.013, length=0.022),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.040, 0.0, 0.123)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    model.articulation(
        "frame_to_lead_screw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lead_screw = object_model.get_part("lead_screw")
    slide = object_model.get_articulation("frame_to_carriage")
    screw_spin = object_model.get_articulation("frame_to_lead_screw")

    ctx.check(
        "linear actuator category matches reference",
        object_model.meta.get("category") == "Robotics / Linear actuator",
        details=f"meta={object_model.meta}",
    )
    ctx.expect_within(
        carriage,
        frame,
        axes="y",
        margin=0.004,
        name="carriage is laterally captured by rail width",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="bearing_shoe_0",
        negative_elem="guide_rod_0",
        name="carriage bearing shoe rides on guide rod",
    )
    ctx.expect_gap(
        carriage,
        lead_screw,
        axis="z",
        min_gap=0.006,
        max_gap=0.018,
        positive_elem="carriage_block",
        negative_elem="screw_core",
        name="lead screw runs below carriage clearance slot",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="xy",
        min_overlap=0.070,
        elem_a="carriage_block",
        elem_b="rail_extrusion",
        name="carriage sits over the straight rail",
    )
    ctx.expect_overlap(
        lead_screw,
        frame,
        axes="x",
        min_overlap=0.560,
        elem_a="screw_core",
        elem_b="rail_extrusion",
        name="lead screw spans most of the rail length",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.180, screw_spin: math.pi / 2.0}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            frame,
            axes="y",
            margin=0.004,
            name="extended carriage remains laterally captured",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.000001,
            positive_elem="bearing_shoe_1",
            negative_elem="guide_rod_1",
            name="extended carriage keeps bearing contact",
        )
    ctx.check(
        "prismatic joint drives carriage along rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.150,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
