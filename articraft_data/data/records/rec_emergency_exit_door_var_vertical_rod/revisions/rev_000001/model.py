from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="double_emergency_exit_doors",
        meta={
            "reference": "source image picture/Emergency Equipment/Emergency exit door/001.png",
            "run_notes": "The reference appears consistent with the category: a pair of gray emergency exit doors with push bars and a metal frame.",
        },
    )

    galvanized = model.material("galvanized_gray", rgba=(0.56, 0.58, 0.56, 1.0))
    darker_metal = model.material("dark_frame_gray", rgba=(0.34, 0.36, 0.35, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.03, 0.035, 0.035, 1.0))
    green = model.material("exit_sign_green", rgba=(0.0, 0.62, 0.28, 1.0))
    white = model.material("sign_white", rgba=(0.94, 0.96, 0.93, 1.0))
    hardware = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))

    door_width = 0.84
    door_height = 2.03
    door_thickness = 0.045
    hinge_x = 0.86
    sill_height = 0.06
    frame_height = 2.19

    frame = model.part("frame")
    # Rectangular steel jamb/header frame, modeled as one connected root assembly.
    frame.visual(
        Box((0.10, 0.11, frame_height)),
        origin=Origin(xyz=(-0.94, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_0",
    )
    frame.visual(
        Box((0.10, 0.11, frame_height)),
        origin=Origin(xyz=(0.94, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_1",
    )
    frame.visual(
        Box((1.98, 0.11, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=darker_metal,
        name="header",
    )
    frame.visual(
        Box((1.98, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=darker_metal,
        name="threshold",
    )
    # Dark exposed hinge/pivot lines on the jambs.
    for idx, x in enumerate((-0.89, 0.89)):
        frame.visual(
            Cylinder(radius=0.012, length=2.02),
            origin=Origin(xyz=(x, 0.040, sill_height + door_height / 2.0)),
            material=shadow,
            name=f"hinge_line_{idx}",
        )

    def add_exit_sign(door, *, x_center: float) -> None:
        door.visual(
            Box((0.34, 0.007, 0.115)),
            origin=Origin(xyz=(x_center, 0.0245, 1.28)),
            material=green,
            name="exit_sign",
        )
        # Simple raised white strokes stand in for the small "Push bar to open" text.
        door.visual(
            Box((0.245, 0.004, 0.018)),
            origin=Origin(xyz=(x_center, 0.0295, 1.295)),
            material=white,
            name="sign_line",
        )
        door.visual(
            Box((0.170, 0.004, 0.014)),
            origin=Origin(xyz=(x_center, 0.0295, 1.258)),
            material=white,
            name="sign_small_line",
        )

    def add_door_leaf(name: str, *, side: int) -> object:
        """Add a metal door leaf.

        side=+1 creates the leaf hinged at negative X and extending inward
        toward the meeting seam. side=-1 mirrors it about the center seam.
        """

        door = model.part(name)
        direction = float(side)
        x_center = direction * door_width / 2.0

        # Main gray slab with subtle raised rails/stiles on the public face.
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(x_center, 0.0, door_height / 2.0)),
            material=galvanized,
            name="door_slab",
        )
        for strip_name, x in (
            ("hinge_stile", direction * 0.035),
            ("meeting_stile", direction * (door_width - 0.035)),
        ):
            door.visual(
                Box((0.052, 0.014, door_height - 0.08)),
                origin=Origin(xyz=(x, 0.0255, door_height / 2.0)),
                material=darker_metal,
                name=strip_name,
            )
        door.visual(
            Box((door_width - 0.08, 0.014, 0.050)),
            origin=Origin(xyz=(x_center, 0.0255, door_height - 0.040)),
            material=darker_metal,
            name="top_rail",
        )
        door.visual(
            Box((door_width - 0.08, 0.014, 0.055)),
            origin=Origin(xyz=(x_center, 0.0255, 0.040)),
            material=darker_metal,
            name="bottom_rail",
        )
        # Black astragal/gasket at the central meeting seam.
        door.visual(
            Box((0.014, 0.012, door_height - 0.03)),
            origin=Origin(xyz=(direction * (door_width - 0.007), 0.029, door_height / 2.0)),
            material=shadow,
            name="meeting_gasket",
        )

        add_exit_sign(door, x_center=x_center)

        # Fixed end housings mounted to the door. The horizontal push bar is a
        # separate moving child part passing just in front of these housings.
        for idx, x in enumerate((x_center - direction * 0.35, x_center + direction * 0.35)):
            door.visual(
                Box((0.055, 0.052, 0.18)),
                origin=Origin(xyz=(x, 0.048, 1.055)),
                material=hardware,
                name=f"bar_mount_{idx}",
            )
            door.visual(
                Box((0.036, 0.040, 0.060)),
                origin=Origin(xyz=(x, 0.046, 1.055)),
                material=shadow,
                name=f"mount_slot_{idx}",
            )

        if side > 0:
            # Stationary rod guide clamps mounted to door_0 face. The actual
            # top/bottom rod bolts are separate moving child parts driven by
            # the push bar via mimic articulations (SVR exit device).
            for clamp_name, z in (
                ("rod_clamp_0", 0.23),
                ("rod_clamp_1", 0.78),
                ("rod_clamp_2", 1.47),
                ("rod_clamp_3", 1.91),
            ):
                door.visual(
                    Box((0.070, 0.048, 0.030)),
                    origin=Origin(xyz=(0.805, 0.046, z)),
                    material=hardware,
                    name=clamp_name,
                )

        return door

    door_0 = add_door_leaf("door_0", side=+1)
    door_1 = add_door_leaf("door_1", side=-1)

    def add_push_bar(name: str) -> object:
        bar = model.part(name)
        bar.visual(
            Cylinder(radius=0.016, length=0.62),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="bar_rail",
        )
        for idx, x in enumerate((-0.315, 0.315)):
            bar.visual(
                Box((0.030, 0.036, 0.036)),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material=hardware,
                name=f"bar_end_{idx}",
            )
        return bar

    push_bar_0 = add_push_bar("push_bar_0")
    push_bar_1 = add_push_bar("push_bar_1")

    # --- Surface vertical-rod (SVR) exit device on door_0 ---
    # Pressing push_bar_0 retracts both rod bolts via mimic articulations.
    # The upper rod pulls the top latch down out of the header; the lower rod
    # lifts the bottom latch up out of the threshold.
    rod_x = 0.805
    rod_y = 0.067
    # Latch bolts sit fully proud of the door stiles/rails (no Y overlap).
    latch_y = 0.068
    rod_retraction = 0.025  # 25 mm latch retraction
    bar_travel = 0.015
    rod_mimic_multiplier = rod_retraction / bar_travel

    # Top rod: upper rod segment + top latch bolt.
    # Rod runs from push-bar height (~1.055) up to near header (~2.03).
    top_rod_len = door_height - 1.055  # ~0.975 m
    top_rod_center_z = 1.055 + top_rod_len / 2.0
    top_latch_z = door_height - 0.025

    top_rod = model.part("top_rod")
    top_rod.visual(
        Cylinder(radius=0.010, length=top_rod_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="top_rod_bar",
    )
    top_rod.visual(
        Box((0.040, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, latch_y - rod_y, top_latch_z - top_rod_center_z)),
        material=hardware,
        name="rod_top_latch",
    )

    # Bottom rod: lower rod segment + bottom latch bolt.
    # Rod runs from sill (~0.06) up to push-bar height (~1.055).
    bottom_rod_len = 1.055 - sill_height  # ~0.995 m
    bottom_rod_center_z = sill_height + bottom_rod_len / 2.0
    bottom_latch_z = 0.045

    bottom_rod = model.part("bottom_rod")
    bottom_rod.visual(
        Cylinder(radius=0.010, length=bottom_rod_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="bottom_rod_bar",
    )
    bottom_rod.visual(
        Box((0.040, 0.055, 0.060)),
        origin=Origin(xyz=(0.0, latch_y - rod_y, bottom_latch_z - bottom_rod_center_z)),
        material=hardware,
        name="rod_bottom_latch",
    )

    # Leaf hinges: closed leaves meet at the center seam; positive motion opens
    # either leaf forward (+Y) independently.
    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, 0.0, sill_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "frame_to_door_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_1,
        origin=Origin(xyz=(hinge_x, 0.0, sill_height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    # Panic bars depress a short distance toward the door face.
    model.articulation(
        "door_0_to_push_bar_0",
        ArticulationType.PRISMATIC,
        parent=door_0,
        child=push_bar_0,
        origin=Origin(xyz=(door_width / 2.0, 0.090, 1.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.015),
    )
    model.articulation(
        "door_1_to_push_bar_1",
        ArticulationType.PRISMATIC,
        parent=door_1,
        child=push_bar_1,
        origin=Origin(xyz=(-door_width / 2.0, 0.090, 1.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.015),
    )

    # SVR rod bolts: pressing push_bar_0 drives both rods to retract.
    # Top rod: positive q moves the latch downward (out of the header).
    model.articulation(
        "door_0_to_top_rod",
        ArticulationType.PRISMATIC,
        parent=door_0,
        child=top_rod,
        origin=Origin(xyz=(rod_x, rod_y, top_rod_center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.15, lower=0.0, upper=rod_retraction,
        ),
        mimic=Mimic(joint="door_0_to_push_bar_0", multiplier=rod_mimic_multiplier),
    )
    # Bottom rod: positive q moves the latch upward (out of the threshold).
    model.articulation(
        "door_0_to_bottom_rod",
        ArticulationType.PRISMATIC,
        parent=door_0,
        child=bottom_rod,
        origin=Origin(xyz=(rod_x, rod_y, bottom_rod_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.15, lower=0.0, upper=rod_retraction,
        ),
        mimic=Mimic(joint="door_0_to_push_bar_0", multiplier=rod_mimic_multiplier),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    push_bar_0 = object_model.get_part("push_bar_0")
    push_bar_1 = object_model.get_part("push_bar_1")
    top_rod = object_model.get_part("top_rod")
    bottom_rod = object_model.get_part("bottom_rod")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    hinge_1 = object_model.get_articulation("frame_to_door_1")
    bar_slide_0 = object_model.get_articulation("door_0_to_push_bar_0")
    bar_slide_1 = object_model.get_articulation("door_1_to_push_bar_1")
    top_rod_joint = object_model.get_articulation("door_0_to_top_rod")
    bottom_rod_joint = object_model.get_articulation("door_0_to_bottom_rod")

    ctx.check(
        "two independent hinged leaves",
        hinge_0.child == "door_0"
        and hinge_1.child == "door_1"
        and hinge_0.mimic is None
        and hinge_1.mimic is None,
        details="Each door leaf should have its own non-mimicked hinge.",
    )
    ctx.check(
        "separate push bar links",
        bar_slide_0.child == "push_bar_0" and bar_slide_1.child == "push_bar_1",
        details="Panic bars should be authored as separate horizontal child parts.",
    )
    ctx.check(
        "SVR rod bolts are separate moving parts on door_0",
        top_rod_joint.parent == "door_0"
        and top_rod_joint.child == "top_rod"
        and bottom_rod_joint.parent == "door_0"
        and bottom_rod_joint.child == "bottom_rod"
        and top_rod_joint.mimic is not None
        and top_rod_joint.mimic.joint == "door_0_to_push_bar_0"
        and bottom_rod_joint.mimic is not None
        and bottom_rod_joint.mimic.joint == "door_0_to_push_bar_0",
        details="Top and bottom rod bolts must be prismatic children of door_0 mimicking the push bar.",
    )

    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.060,
        name="closed leaves retain a narrow central seam",
    )
    ctx.expect_within(
        door_0,
        frame,
        axes="xz",
        margin=0.005,
        name="door_0 sits inside the outer frame envelope",
    )
    ctx.expect_within(
        door_1,
        frame,
        axes="xz",
        margin=0.005,
        name="door_1 sits inside the outer frame envelope",
    )
    ctx.expect_overlap(
        push_bar_0,
        door_0,
        axes="x",
        elem_a="bar_rail",
        elem_b="door_slab",
        min_overlap=0.20,
        name="push_bar_0 spans across door_0",
    )
    ctx.expect_overlap(
        push_bar_1,
        door_1,
        axes="x",
        elem_a="bar_rail",
        elem_b="door_slab",
        min_overlap=0.20,
        name="push_bar_1 spans across door_1",
    )

    rest_0 = ctx.part_world_aabb(door_0)
    rest_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.05}):
        opened_0 = ctx.part_world_aabb(door_0)
    with ctx.pose({hinge_1: 1.05}):
        opened_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "door_0 swings forward",
        rest_0 is not None and opened_0 is not None and opened_0[1][1] > rest_0[1][1] + 0.45,
        details=f"rest={rest_0}, opened={opened_0}",
    )
    ctx.check(
        "door_1 swings forward independently",
        rest_1 is not None and opened_1 is not None and opened_1[1][1] > rest_1[1][1] + 0.45,
        details=f"rest={rest_1}, opened={opened_1}",
    )

    rest_bar = ctx.part_world_position(push_bar_0)
    # Pose the source bar joint; mimic followers follow automatically.
    with ctx.pose({bar_slide_0: 0.015, bar_slide_1: 0.015}):
        depressed_bar = ctx.part_world_position(push_bar_0)
    ctx.check(
        "push bars depress toward the door",
        rest_bar is not None and depressed_bar is not None and depressed_bar[1] < rest_bar[1] - 0.010,
        details=f"rest={rest_bar}, depressed={depressed_bar}",
    )

    # SVR rod bars slide through the guide clamps mounted on door_0 — this is
    # the intended captured-shaft-in-bracket relationship of a real SVR exit
    # device. Scope allowances to the reported element pairs.
    ctx.allow_overlap(
        door_0,
        bottom_rod,
        elem_a="rod_clamp_0",
        elem_b="bottom_rod_bar",
        reason="Bottom rod bar slides through guide clamp as captured-shaft fit.",
    )
    ctx.allow_overlap(
        door_0,
        bottom_rod,
        elem_a="rod_clamp_1",
        elem_b="bottom_rod_bar",
        reason="Bottom rod bar slides through guide clamp as captured-shaft fit.",
    )
    ctx.allow_overlap(
        door_0,
        top_rod,
        elem_a="rod_clamp_2",
        elem_b="top_rod_bar",
        reason="Top rod bar slides through guide clamp as captured-shaft fit.",
    )
    ctx.allow_overlap(
        door_0,
        top_rod,
        elem_a="rod_clamp_3",
        elem_b="top_rod_bar",
        reason="Top rod bar slides through guide clamp as captured-shaft fit.",
    )

    # Prove the rod bars remain engaged with the clamps.
    ctx.expect_contact(
        door_0,
        bottom_rod,
        elem_a="rod_clamp_0",
        elem_b="bottom_rod_bar",
        name="bottom rod bar is guided through clamp_0",
    )
    ctx.expect_contact(
        door_0,
        top_rod,
        elem_a="rod_clamp_2",
        elem_b="top_rod_bar",
        name="top rod bar is guided through clamp_2",
    )

    # SVR mechanism: pressing push_bar_0 retracts both rod bolts.
    rest_top = ctx.part_world_position(top_rod)
    rest_bottom = ctx.part_world_position(bottom_rod)
    with ctx.pose({bar_slide_0: 0.015}):
        retracted_top = ctx.part_world_position(top_rod)
        retracted_bottom = ctx.part_world_position(bottom_rod)
    ctx.check(
        "top rod bolt retracts downward when bar is pressed",
        rest_top is not None
        and retracted_top is not None
        and retracted_top[2] < rest_top[2] - 0.010,
        details=f"rest={rest_top}, retracted={retracted_top}",
    )
    ctx.check(
        "bottom rod bolt retracts upward when bar is pressed",
        rest_bottom is not None
        and retracted_bottom is not None
        and retracted_bottom[2] > rest_bottom[2] + 0.010,
        details=f"rest={rest_bottom}, retracted={retracted_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
