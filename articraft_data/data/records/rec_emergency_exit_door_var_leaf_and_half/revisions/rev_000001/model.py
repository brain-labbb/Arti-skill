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
        name="leaf_and_half_emergency_exit_doors",
        meta={
            "reference": "source image picture/Emergency Equipment/Emergency exit door/001.png",
            "run_notes": (
                "Leaf-and-a-half fire exit configuration: one wide active leaf "
                "with panic bar and one narrow normally-bolted inactive leaf."
            ),
        },
    )

    galvanized = model.material("galvanized_gray", rgba=(0.56, 0.58, 0.56, 1.0))
    darker_metal = model.material("dark_frame_gray", rgba=(0.34, 0.36, 0.35, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.03, 0.035, 0.035, 1.0))
    green = model.material("exit_sign_green", rgba=(0.0, 0.62, 0.28, 1.0))
    white = model.material("sign_white", rgba=(0.94, 0.96, 0.93, 1.0))
    hardware = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))

    # Leaf-and-a-half widths: wide active leaf + narrow inactive leaf fill the
    # frame opening between hinge pivots (±0.86 m) with a ~0.04 m seam gap.
    door_0_width = 1.15
    door_1_width = 0.53
    door_height = 2.03
    door_thickness = 0.045
    hinge_x = 0.86
    sill_height = 0.06
    frame_height = 2.19

    # ── Frame (root) ────────────────────────────────────────────────────────
    frame = model.part("frame")
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
    for idx, x in enumerate((-0.89, 0.89)):
        frame.visual(
            Cylinder(radius=0.012, length=2.02),
            origin=Origin(xyz=(x, 0.040, sill_height + door_height / 2.0)),
            material=shadow,
            name=f"hinge_line_{idx}",
        )

    # ── Exit sign helper ────────────────────────────────────────────────────
    def add_exit_sign(door, *, x_center: float) -> None:
        door.visual(
            Box((0.34, 0.007, 0.115)),
            origin=Origin(xyz=(x_center, 0.0245, 1.28)),
            material=green,
            name="exit_sign",
        )
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

    # ── Door leaf helper (parameterised by width and active/inactive) ───────
    def add_door_leaf(
        name: str, *, side: int, width: float, active: bool
    ) -> object:
        """Add a metal door leaf.

        side=+1 creates the leaf hinged at negative X and extending toward the
        meeting seam.  side=-1 mirrors about the seam.
        active=True adds panic-bar mounts and (for the +side leaf) the visible
        vertical locking rod.  active=False adds flush-bolt housings at the
        top and bottom rails instead.
        """
        door = model.part(name)
        direction = float(side)
        x_center = direction * width / 2.0

        # Main gray slab.
        door.visual(
            Box((width, door_thickness, door_height)),
            origin=Origin(xyz=(x_center, 0.0, door_height / 2.0)),
            material=galvanized,
            name="door_slab",
        )
        # Raised hinge stile and meeting stile on the public face.
        for strip_name, x in (
            ("hinge_stile", direction * 0.035),
            ("meeting_stile", direction * (width - 0.035)),
        ):
            door.visual(
                Box((0.052, 0.014, door_height - 0.08)),
                origin=Origin(xyz=(x, 0.0255, door_height / 2.0)),
                material=darker_metal,
                name=strip_name,
            )
        # Top and bottom rails.
        door.visual(
            Box((width - 0.08, 0.014, 0.050)),
            origin=Origin(xyz=(x_center, 0.0255, door_height - 0.040)),
            material=darker_metal,
            name="top_rail",
        )
        door.visual(
            Box((width - 0.08, 0.014, 0.055)),
            origin=Origin(xyz=(x_center, 0.0255, 0.040)),
            material=darker_metal,
            name="bottom_rail",
        )
        # Black astragal / meeting gasket at the meeting seam edge.
        door.visual(
            Box((0.014, 0.012, door_height - 0.03)),
            origin=Origin(
                xyz=(direction * (width - 0.007), 0.029, door_height / 2.0)
            ),
            material=shadow,
            name="meeting_gasket",
        )

        add_exit_sign(door, x_center=x_center)

        if active:
            # Panic-bar end housings mounted to the active leaf face.
            mount_offset = width * 0.30
            for idx, x in enumerate(
                (x_center - direction * mount_offset, x_center + direction * mount_offset)
            ):
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
                # Vertical locking rod beside the meeting seam (active leaf).
                rod_x = width - 0.035
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
        else:
            # Inactive leaf: flush-bolt housings at top and bottom meeting-stile
            # edge, with short bolt pins into the header and threshold.
            bolt_x = direction * (width - 0.030)
            # Top flush bolt housing + short pin (clear of header).
            door.visual(
                Box((0.032, 0.030, 0.14)),
                origin=Origin(xyz=(bolt_x, 0.025, door_height - 0.10)),
                material=hardware,
                name="flush_bolt_top",
            )
            door.visual(
                Cylinder(radius=0.005, length=0.025),
                origin=Origin(xyz=(bolt_x, 0.025, door_height - 0.02)),
                material=hardware,
                name="flush_bolt_pin_top",
            )
            # Bottom flush bolt housing + short pin (clear of threshold).
            door.visual(
                Box((0.032, 0.030, 0.14)),
                origin=Origin(xyz=(bolt_x, 0.025, 0.14)),
                material=hardware,
                name="flush_bolt_bottom",
            )
            door.visual(
                Cylinder(radius=0.005, length=0.025),
                origin=Origin(xyz=(bolt_x, 0.025, 0.055)),
                material=hardware,
                name="flush_bolt_pin_bottom",
            )

        return door

    door_0 = add_door_leaf("door_0", side=+1, width=door_0_width, active=True)
    door_1 = add_door_leaf("door_1", side=-1, width=door_1_width, active=False)

    # ── Push bar (active leaf only) ─────────────────────────────────────────
    def add_push_bar(name: str, *, rail_length: float) -> object:
        bar = model.part(name)
        bar.visual(
            Cylinder(radius=0.016, length=rail_length),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hardware,
            name="bar_rail",
        )
        end_offset = rail_length / 2.0 - 0.005
        for idx, x in enumerate((-end_offset, end_offset)):
            bar.visual(
                Box((0.030, 0.036, 0.036)),
                origin=Origin(xyz=(x, 0.0, 0.0)),
                material=hardware,
                name=f"bar_end_{idx}",
            )
        return bar

    push_bar_0 = add_push_bar("push_bar_0", rail_length=0.82)

    # ── Articulations ───────────────────────────────────────────────────────
    # Leaf hinges: closed leaves meet at the offset seam; positive motion opens
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

    # Active panic bar depresses a short distance toward the door face.
    model.articulation(
        "door_0_to_push_bar_0",
        ArticulationType.PRISMATIC,
        parent=door_0,
        child=push_bar_0,
        origin=Origin(xyz=(door_0_width / 2.0, 0.090, 1.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.20, lower=0.0, upper=0.015),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    push_bar_0 = object_model.get_part("push_bar_0")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    hinge_1 = object_model.get_articulation("frame_to_door_1")
    bar_slide_0 = object_model.get_articulation("door_0_to_push_bar_0")

    # ── Structural delta: unequal leaf widths ──────────────────────────────
    door_0_dims = ctx.part_world_aabb(door_0)
    door_1_dims = ctx.part_world_aabb(door_1)
    ctx.check(
        "door_0 is the wide active leaf",
        door_0_dims is not None
        and (door_0_dims[1][0] - door_0_dims[0][0]) > 0.90,
        details=f"door_0 x-span should exceed 0.90 m; got {door_0_dims}",
    )
    ctx.check(
        "door_1 is the narrow inactive leaf",
        door_1_dims is not None
        and (door_1_dims[1][0] - door_1_dims[0][0]) < 0.75,
        details=f"door_1 x-span should be under 0.75 m; got {door_1_dims}",
    )
    ctx.check(
        "active leaf is at least 1.5x wider than inactive leaf",
        door_0_dims is not None
        and door_1_dims is not None
        and (door_0_dims[1][0] - door_0_dims[0][0])
        > 1.5 * (door_1_dims[1][0] - door_1_dims[0][0]),
        details="Leaf-and-a-half ratio should be at least 1.5:1.",
    )

    # ── Flush bolts on inactive leaf ────────────────────────────────────────
    door_1_visuals = {v.name for v in door_1.visuals}
    ctx.check(
        "door_1 has flush bolt housings instead of a push bar",
        "flush_bolt_top" in door_1_visuals and "flush_bolt_bottom" in door_1_visuals,
        details="Inactive leaf should carry top and bottom flush-bolt housings.",
    )

    # ── No push bar on inactive leaf ────────────────────────────────────────
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no push_bar_1 part exists (inactive leaf has no panic bar)",
        "push_bar_1" not in part_names,
        details="Only the active leaf should carry a push bar.",
    )

    # ── Hinge and push-bar articulation checks ─────────────────────────────
    ctx.check(
        "two independent revolute hinges remain",
        hinge_0.child == "door_0"
        and hinge_1.child == "door_1"
        and hinge_0.mimic is None
        and hinge_1.mimic is None,
        details="Each leaf should have its own non-mimicked revolute hinge.",
    )
    ctx.check(
        "push_bar_0 is a separate prismatic child of door_0",
        bar_slide_0.child == "push_bar_0",
        details="Active panic bar should be a separate prismatic child part.",
    )

    # ── Geometric fit ──────────────────────────────────────────────────────
    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.060,
        name="closed leaves retain a narrow offset seam",
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
        name="push_bar_0 spans across the wide active door_0",
    )

    # ── Articulated-pose checks ─────────────────────────────────────────────
    rest_0 = ctx.part_world_aabb(door_0)
    rest_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({hinge_0: 1.05}):
        opened_0 = ctx.part_world_aabb(door_0)
    with ctx.pose({hinge_1: 1.05}):
        opened_1 = ctx.part_world_aabb(door_1)
    ctx.check(
        "door_0 (wide active leaf) swings forward",
        rest_0 is not None and opened_0 is not None and opened_0[1][1] > rest_0[1][1] + 0.45,
        details=f"rest={rest_0}, opened={opened_0}",
    )
    ctx.check(
        "door_1 (narrow inactive leaf) swings forward independently",
        rest_1 is not None and opened_1 is not None and opened_1[1][1] > rest_1[1][1] + 0.25,
        details=f"rest={rest_1}, opened={opened_1}",
    )

    rest_bar = ctx.part_world_position(push_bar_0)
    with ctx.pose({bar_slide_0: 0.015}):
        depressed_bar = ctx.part_world_position(push_bar_0)
    ctx.check(
        "push_bar_0 depresses toward the door face",
        rest_bar is not None and depressed_bar is not None and depressed_bar[1] < rest_bar[1] - 0.010,
        details=f"rest={rest_bar}, depressed={depressed_bar}",
    )

    return ctx.report()


object_model = build_object_model()
