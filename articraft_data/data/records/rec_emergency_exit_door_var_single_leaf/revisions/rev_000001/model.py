from __future__ import annotations

from math import pi

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
        name="single_emergency_exit_door",
        meta={
            "reference": "source image picture/Emergency Equipment/Emergency exit door/001.png",
            "run_notes": "Single-leaf emergency exit door variant derived from the paired-leaf parent. "
            "One hinged steel egress leaf with panic bar, exit sign, and steel frame. "
            "The single leaf strikes into the fixed right jamb instead of a mating leaf.",
        },
    )

    galvanized = model.material("galvanized_gray", rgba=(0.56, 0.58, 0.56, 1.0))
    darker_metal = model.material("dark_frame_gray", rgba=(0.34, 0.36, 0.35, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.03, 0.035, 0.035, 1.0))
    green = model.material("exit_sign_green", rgba=(0.0, 0.62, 0.28, 1.0))
    white = model.material("sign_white", rgba=(0.94, 0.96, 0.93, 1.0))
    hardware = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))

    # Single-leaf door dimensions (~0.95 m clear opening).
    door_width = 0.93
    door_height = 2.03
    door_thickness = 0.045
    sill_height = 0.06
    frame_height = 2.19

    # Frame geometry: jamb inner faces define the clear opening.
    single_opening = 0.95
    jamb_width = 0.10
    jamb_depth = 0.11
    half_opening = single_opening / 2.0
    jamb_x = half_opening + jamb_width / 2.0  # jamb center offset from frame center
    frame_span = single_opening + 2.0 * jamb_width  # total outer frame width

    # Hinge pivot sits at the inner face of the left jamb.
    hinge_x = half_opening

    frame = model.part("frame")
    # Rectangular steel jamb/header frame, modeled as one connected root assembly.
    frame.visual(
        Box((jamb_width, jamb_depth, frame_height)),
        origin=Origin(xyz=(-jamb_x, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_0",
    )
    frame.visual(
        Box((jamb_width, jamb_depth, frame_height)),
        origin=Origin(xyz=(jamb_x, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_1",
    )
    frame.visual(
        Box((frame_span, jamb_depth, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=darker_metal,
        name="header",
    )
    frame.visual(
        Box((frame_span, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=darker_metal,
        name="threshold",
    )
    # Single hinge pivot line on the left jamb only.
    frame.visual(
        Cylinder(radius=0.012, length=2.02),
        origin=Origin(xyz=(-(half_opening + 0.015), 0.040, sill_height + door_height / 2.0)),
        material=shadow,
        name="hinge_line_0",
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
        """Add a single metal door leaf.

        side=+1 creates the leaf hinged at negative X (left jamb) extending
        toward the strike jamb. The meeting_stile now represents the lock/strike
        stile on the free edge.
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

        add_exit_sign(door, x_center=x_center)

        # Fixed end housings mounted to the door. The horizontal push bar is a
        # separate moving child part passing just in front of these housings.
        mount_offset = 0.35 * (door_width / 0.84)
        for idx, x in enumerate((x_center - direction * mount_offset, x_center + direction * mount_offset)):
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
            # The single leaf has the visible vertical locking rod on the
            # strike/lock side (the free edge that meets the right jamb).
            rod_x = door_width - 0.035
            door.visual(
                Cylinder(radius=0.010, length=1.86),
                origin=Origin(xyz=(rod_x, 0.067, door_height / 2.0)),
                material=hardware,
                name="vertical_rod",
            )
            for idx, z in enumerate((0.23, 0.78, 1.47, 1.91)):
                door.visual(
                    Box((0.070, 0.048, 0.030)),
                    origin=Origin(xyz=(rod_x, 0.046, z)),
                    material=hardware,
                    name=f"rod_clamp_{idx}",
                )
            door.visual(
                Box((0.040, 0.055, 0.060)),
                origin=Origin(xyz=(rod_x, 0.050, door_height - 0.025)),
                material=hardware,
                name="rod_top_latch",
            )
            door.visual(
                Box((0.040, 0.055, 0.060)),
                origin=Origin(xyz=(rod_x, 0.050, 0.045)),
                material=hardware,
                name="rod_bottom_latch",
            )

        return door

    # Single active leaf only.
    door_0 = add_door_leaf("door_0", side=+1)

    def add_push_bar(name: str) -> object:
        bar = model.part(name)
        bar.visual(
            Cylinder(radius=0.016, length=0.70),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="bar_rail",
        )
        for idx, x in enumerate((-0.355, 0.355)):
            bar.visual(
                Box((0.030, 0.036, 0.036)),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material=hardware,
                name=f"bar_end_{idx}",
            )
        return bar

    push_bar_0 = add_push_bar("push_bar_0")

    # Single leaf hinge: closed leaf fills the opening; positive motion opens
    # the leaf forward (+Y) swinging about the left jamb pivot.
    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, 0.0, sill_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.55),
    )

    # Panic bar depresses a short distance toward the door face.
    model.articulation(
        "door_0_to_push_bar_0",
        ArticulationType.PRISMATIC,
        parent=door_0,
        child=push_bar_0,
        origin=Origin(xyz=(door_width / 2.0, 0.090, 1.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    push_bar_0 = object_model.get_part("push_bar_0")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    bar_slide_0 = object_model.get_articulation("door_0_to_push_bar_0")

    # Structural delta: exactly one active leaf with its own hinge and push bar.
    ctx.check(
        "single hinged leaf (door_0 only)",
        hinge_0.child == "door_0" and hinge_0.mimic is None,
        details="The single-leaf variant should have exactly one non-mimicked hinge on door_0.",
    )
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no second door or push bar",
        "door_1" not in part_names and "push_bar_1" not in part_names,
        details="The paired-leaf parts (door_1, push_bar_1) must be removed in the single-leaf variant.",
    )
    ctx.check(
        "single push bar link",
        bar_slide_0.child == "push_bar_0",
        details="Panic bar should be authored as a separate horizontal child part on door_0.",
    )

    ctx.expect_within(
        door_0,
        frame,
        axes="xz",
        margin=0.005,
        name="door_0 sits inside the outer frame envelope",
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

    # Single leaf should strike into the fixed right jamb (jamb_1) at rest.
    ctx.expect_gap(
        frame,
        door_0,
        axis="x",
        positive_elem="jamb_1",
        negative_elem="door_slab",
        min_gap=-0.005,
        max_gap=0.060,
        name="single leaf free edge meets the right jamb",
    )

    rest_0 = ctx.part_world_aabb(door_0)
    with ctx.pose({hinge_0: 1.05}):
        opened_0 = ctx.part_world_aabb(door_0)
    ctx.check(
        "door_0 swings forward",
        rest_0 is not None and opened_0 is not None and opened_0[1][1] > rest_0[1][1] + 0.45,
        details=f"rest={rest_0}, opened={opened_0}",
    )

    rest_bar = ctx.part_world_position(push_bar_0)
    with ctx.pose({bar_slide_0: 0.015}):
        depressed_bar = ctx.part_world_position(push_bar_0)
    ctx.check(
        "push bar depresses toward the door",
        rest_bar is not None
        and depressed_bar is not None
        and depressed_bar[1] < rest_bar[1] - 0.010,
        details=f"rest={rest_bar}, depressed={depressed_bar}",
    )

    return ctx.report()


object_model = build_object_model()
