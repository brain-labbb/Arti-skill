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
        name="telescoping_linear_actuator",
        meta={
            "category": "Robotics / Linear actuator",
            "reference_note": "Telescoping nested-tube linear actuator with 3 prismatic stages driven by a central lead screw.",
        },
    )

    aluminum = model.material("brushed_aluminum", color=(0.72, 0.72, 0.68, 1.0))
    dark_aluminum = model.material("black_anodized", color=(0.015, 0.016, 0.018, 1.0))
    steel = model.material("polished_steel", color=(0.82, 0.84, 0.82, 1.0))
    screw_steel = model.material("lead_screw_steel", color=(0.64, 0.66, 0.64, 1.0))

    frame = model.part("frame")

    # Motor block at the base (keep from parent)
    frame.visual(
        Box((0.084, 0.128, 0.095)),
        origin=Origin(xyz=(-0.062, 0.0, 0.0)),
        material=dark_aluminum,
        name="motor_block",
    )
    frame.visual(
        Box((0.090, 0.106, 0.040)),
        origin=Origin(xyz=(-0.062, 0.0, 0.068)),
        material=dark_aluminum,
        name="motor_cap",
    )
    # Mounting bracket connecting motor block to base tube (extends to overlap the tube wall for connectivity)
    frame.visual(
        Box((0.055, 0.100, 0.096)),
        origin=Origin(xyz=(-0.013, 0.0, 0.0)),
        material=dark_aluminum,
        name="motor_bracket",
    )
    # Upper bracket connecting motor cap to base tube
    frame.visual(
        Box((0.050, 0.050, 0.048)),
        origin=Origin(xyz=(-0.010, 0.0, 0.058)),
        material=dark_aluminum,
        name="motor_cap_bracket",
    )

    # Helper function to add tube visuals to a part
    def add_tube_visuals(part, radius, length, material, name_prefix, is_base=False):
        # Main tube body - positioned so it extends from part origin along +X
        tube_name = "rail_extrusion" if is_base else f"{name_prefix}_tube"
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(length / 2.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=tube_name,
        )
        # Base flange/cap
        part.visual(
            Cylinder(radius=radius * 1.08, length=0.018),
            origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_aluminum,
            name=f"{name_prefix}_base_flange",
        )
        # Tip ring
        part.visual(
            Cylinder(radius=radius * 1.02, length=0.012),
            origin=Origin(xyz=(length - 0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_aluminum,
            name=f"{name_prefix}_tip_ring",
        )

    # Telescoping stages: 3 nested tubes with decreasing cross-section and length
    # stage_0: outermost/largest tube, fixed to frame
    # stage_1: middle tube, prismatic child of frame via stage_slide_1
    # stage_2: innermost/smallest tube (moving rod), prismatic child of stage_1 via stage_slide_2
    stage_params = [
        {"radius": 0.048, "length": 0.50, "material": aluminum, "slide_upper": None},  # stage_0
        {"radius": 0.040, "length": 0.42, "material": aluminum, "slide_upper": 0.15},  # stage_1
        {"radius": 0.032, "length": 0.34, "material": steel, "slide_upper": 0.12},     # stage_2
    ]

    stages = []
    for i, params in enumerate(stage_params):
        radius = params["radius"]
        length = params["length"]
        material = params["material"]
        slide_upper = params["slide_upper"]

        if i == 0:
            # Base tube visuals on frame (fixed to frame)
            add_tube_visuals(frame, radius, length, material, f"stage_{i}", is_base=True)
            stages.append(frame)
        else:
            # Create stage part
            stage = model.part(f"stage_{i}")
            add_tube_visuals(stage, radius, length, material, f"stage_{i}")
            stages.append(stage)

            # Add PRISMATIC joint: stage_i slides along +X relative to parent
            parent = stages[i - 1]
            model.articulation(
                f"stage_slide_{i}",
                ArticulationType.PRISMATIC,
                parent=parent,
                child=stage,
                origin=Origin(xyz=(0.05, 0.0, 0.0)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(
                    effort=160.0,
                    velocity=0.35,
                    lower=0.0,
                    upper=slide_upper,
                ),
            )

    # Lead screw runs through the center of the telescoping tubes (keep from parent)
    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        Cylinder(radius=0.006, length=0.58),
        origin=Origin(xyz=(0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_steel,
        name="screw_core",
    )
    # Thread crests for visual realism
    for i in range(28):
        x = 0.02 + i * 0.02
        lead_screw.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_steel,
            name=f"thread_crest_{i:02d}",
        )

    # Lead screw articulation (keep from parent)
    model.articulation(
        "frame_to_lead_screw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lead_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    lead_screw = object_model.get_part("lead_screw")

    stage_slide_1 = object_model.get_articulation("stage_slide_1")
    stage_slide_2 = object_model.get_articulation("stage_slide_2")
    screw_spin = object_model.get_articulation("frame_to_lead_screw")

    # Verify telescoping stages exist
    ctx.check(
        "telescoping stages exist",
        stage_1 is not None and stage_2 is not None,
        details="stage_1 and stage_2 parts must exist for telescoping topology",
    )

    # Verify stage_slide joints exist
    ctx.check(
        "stage_slide joints exist",
        stage_slide_1 is not None and stage_slide_2 is not None,
        details="stage_slide_1 and stage_slide_2 articulations must exist",
    )

    # Intentional overlaps: nested telescoping tubes and central lead screw
    # These are mechanically valid - inner tubes slide inside outer tubes with guide rings at ends
    ctx.allow_overlap(
        frame,
        stage_1,
        elem_a="rail_extrusion",
        elem_b="stage_1_tube",
        reason="stage_1 tube intentionally nests inside frame base tube for telescoping sliding fit",
    )
    ctx.allow_overlap(
        frame,
        stage_1,
        elem_a="rail_extrusion",
        elem_b="stage_1_base_flange",
        reason="stage_1 base flange sits at the entrance of the frame base tube as a sliding guide ring",
    )
    ctx.allow_overlap(
        frame,
        stage_1,
        elem_a="rail_extrusion",
        elem_b="stage_1_tip_ring",
        reason="stage_1 tip ring acts as a guide bearing at the exit of the frame base tube",
    )
    ctx.allow_overlap(
        frame,
        stage_2,
        elem_a="rail_extrusion",
        elem_b="stage_2_tube",
        reason="stage_2 tube intentionally nests inside frame base tube (passes through stage_1) for telescoping sliding fit",
    )
    ctx.allow_overlap(
        frame,
        stage_2,
        elem_a="rail_extrusion",
        elem_b="stage_2_base_flange",
        reason="stage_2 base flange sits at the entrance of the frame base tube as a sliding guide ring",
    )
    ctx.allow_overlap(
        frame,
        stage_2,
        elem_a="rail_extrusion",
        elem_b="stage_2_tip_ring",
        reason="stage_2 tip ring acts as a guide bearing at the exit of the frame base tube",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_1_tube",
        elem_b="stage_2_tube",
        reason="stage_2 tube intentionally nests inside stage_1 tube for telescoping sliding fit",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_1_tube",
        elem_b="stage_2_base_flange",
        reason="stage_2 base flange sits at the entrance of stage_1 tube as a sliding guide ring",
    )
    ctx.allow_overlap(
        stage_1,
        stage_2,
        elem_a="stage_1_tube",
        elem_b="stage_2_tip_ring",
        reason="stage_2 tip ring acts as a guide bearing at the exit of stage_1 tube",
    )
    ctx.allow_overlap(
        frame,
        lead_screw,
        elem_a="rail_extrusion",
        elem_b="screw_core",
        reason="lead screw runs through the center of the base tube to drive the telescoping stages",
    )
    ctx.allow_overlap(
        frame,
        lead_screw,
        elem_a="stage_0_base_flange",
        elem_b="screw_core",
        reason="lead screw passes through stage_0 base flange center bore at tube entrance",
    )
    ctx.allow_overlap(
        frame,
        lead_screw,
        elem_a="stage_0_tip_ring",
        elem_b="screw_core",
        reason="lead screw passes through stage_0 tip ring center bore at tube exit",
    )
    ctx.allow_overlap(
        frame,
        lead_screw,
        elem_a="motor_bracket",
        elem_b="screw_core",
        reason="lead screw passes through the motor mounting bracket at the base of the actuator",
    )
    ctx.allow_overlap(
        stage_1,
        lead_screw,
        elem_a="stage_1_tube",
        elem_b="screw_core",
        reason="lead screw passes through stage_1 tube center to drive telescoping motion",
    )
    ctx.allow_overlap(
        stage_1,
        lead_screw,
        elem_a="stage_1_base_flange",
        elem_b="screw_core",
        reason="lead screw passes through stage_1 base flange center bore to drive telescoping motion",
    )
    ctx.allow_overlap(
        stage_1,
        lead_screw,
        elem_a="stage_1_tip_ring",
        elem_b="screw_core",
        reason="lead screw passes through stage_1 tip ring center bore to drive telescoping motion",
    )
    ctx.allow_overlap(
        stage_2,
        lead_screw,
        elem_a="stage_2_tube",
        elem_b="screw_core",
        reason="lead screw passes through stage_2 tube center to drive telescoping motion",
    )
    ctx.allow_overlap(
        stage_2,
        lead_screw,
        elem_a="stage_2_base_flange",
        elem_b="screw_core",
        reason="lead screw passes through stage_2 base flange center bore to drive telescoping motion",
    )
    ctx.allow_overlap(
        stage_2,
        lead_screw,
        elem_a="stage_2_tip_ring",
        elem_b="screw_core",
        reason="lead screw passes through stage_2 tip ring center bore to drive telescoping motion",
    )

    # Verify nesting: inner stages are within outer stages on YZ axes
    ctx.expect_within(
        stage_1,
        frame,
        axes="yz",
        margin=0.005,
        name="stage_1 nests within frame base tube cross-section",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="yz",
        margin=0.005,
        name="stage_2 nests within stage_1 cross-section",
    )

    # Verify stages extend along +X when articulated
    rest_pos_1 = ctx.part_world_position(stage_1)
    rest_pos_2 = ctx.part_world_position(stage_2)

    with ctx.pose({stage_slide_1: 0.15, stage_slide_2: 0.12}):
        extended_pos_1 = ctx.part_world_position(stage_1)
        extended_pos_2 = ctx.part_world_position(stage_2)

        ctx.check(
            "stage_1 extends along +X via stage_slide_1",
            rest_pos_1 is not None and extended_pos_1 is not None and extended_pos_1[0] > rest_pos_1[0] + 0.10,
            details=f"rest={rest_pos_1}, extended={extended_pos_1}",
        )

        ctx.check(
            "stage_2 extends along +X via stage_slide_2",
            rest_pos_2 is not None and extended_pos_2 is not None and extended_pos_2[0] > rest_pos_2[0] + 0.08,
            details=f"rest={rest_pos_2}, extended={extended_pos_2}",
        )

        # Verify nesting remains valid at extended pose
        ctx.expect_within(
            stage_1,
            frame,
            axes="yz",
            margin=0.005,
            name="stage_1 remains nested in frame at full extension",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.005,
            name="stage_2 remains nested in stage_1 at full extension",
        )

    # Verify lead screw still exists and rotates
    ctx.check(
        "lead screw exists",
        lead_screw is not None,
        details="lead_screw part must exist",
    )
    ctx.check(
        "frame_to_lead_screw articulation exists",
        screw_spin is not None,
        details="frame_to_lead_screw articulation must exist",
    )

    return ctx.report()


object_model = build_object_model()
