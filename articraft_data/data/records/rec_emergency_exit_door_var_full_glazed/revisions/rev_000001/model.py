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
        name="double_emergency_exit_doors",
        meta={
            "reference": "source image picture/Emergency Equipment/Emergency exit door/001.png",
            "run_notes": (
                "Glazed stile-and-rail variant of the emergency exit door pair. "
                "Solid galvanized slabs replaced with dark bronze anodized aluminum "
                "frame members around a large bronze-tinted tempered glass infill. "
                "All panic hardware, exit signs, hinges, and frame preserved."
            ),
        },
    )

    # --- Materials (companion colorway: bronze glass / dark anodized frame) ---
    anodized_frame = model.material("anodized_aluminum", rgba=(0.42, 0.40, 0.38, 1.0))
    darker_metal = model.material("dark_frame_gray", rgba=(0.34, 0.36, 0.35, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.03, 0.035, 0.035, 1.0))
    green = model.material("exit_sign_green", rgba=(0.0, 0.62, 0.28, 1.0))
    white = model.material("sign_white", rgba=(0.94, 0.96, 0.93, 1.0))
    hardware = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))
    tinted_glass = model.material("bronze_glass", rgba=(0.52, 0.42, 0.32, 0.40))

    # --- Shared dimensions ---
    door_width = 0.84
    door_height = 2.03
    stile_width = 0.065
    rail_height = 0.100
    frame_depth = 0.044
    glass_thickness = 0.010
    hinge_x = 0.86
    sill_height = 0.06
    frame_height = 2.19

    # Glass infill overlaps stiles/rails by 2mm on each side (glazing pocket seat).
    glass_overlap = 0.002
    glass_width = door_width - 2.0 * stile_width + 2.0 * glass_overlap
    glass_height = door_height - 2.0 * rail_height + 2.0 * glass_overlap

    # ==============================
    # Frame (root)
    # ==============================
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

    # ==============================
    # Exit sign (inline helper)
    # ==============================
    def add_exit_sign(door, *, x_center: float) -> None:
        # Mounted on the front glass face. Glass front at y = +glass_thickness/2 = 0.005.
        # Exit sign overlaps the glass for connectivity.
        door.visual(
            Box((0.34, 0.007, 0.115)),
            origin=Origin(xyz=(x_center, 0.005, 1.28)),
            material=green,
            name="exit_sign",
        )
        # Text lines sit proud of the sign face, overlapping exit_sign for connectivity.
        door.visual(
            Box((0.245, 0.004, 0.018)),
            origin=Origin(xyz=(x_center, 0.009, 1.295)),
            material=white,
            name="sign_line",
        )
        door.visual(
            Box((0.170, 0.004, 0.014)),
            origin=Origin(xyz=(x_center, 0.009, 1.258)),
            material=white,
            name="sign_small_line",
        )

    # ==============================
    # Glazed stile-and-rail door leaf
    # ==============================
    def add_door_leaf(name: str, *, side: int) -> object:
        """Add a fully glazed stile-and-rail exit leaf.

        Narrow anodized aluminum perimeter stiles and rails frame a large
        bronze-tinted tempered glass infill. The push bar and exit sign mount
        onto the glass / rails.

        side=+1 creates the leaf hinged at negative X and extending toward
        the meeting seam. side=-1 mirrors it about the center seam.
        """
        door = model.part(name)
        direction = float(side)
        x_center = direction * door_width / 2.0

        # --- Large transparent glass infill (seated in glazing pocket) ---
        door.visual(
            Box((glass_width, glass_thickness, glass_height)),
            origin=Origin(xyz=(x_center, 0.0, door_height / 2.0)),
            material=tinted_glass,
            name="glass_infill",
        )

        # --- Narrow perimeter stiles (full height, structural frame) ---
        for strip_name, x in (
            ("hinge_stile", direction * (stile_width / 2.0)),
            ("meeting_stile", direction * (door_width - stile_width / 2.0)),
        ):
            door.visual(
                Box((stile_width, frame_depth, door_height)),
                origin=Origin(xyz=(x, 0.0, door_height / 2.0)),
                material=anodized_frame,
                name=strip_name,
            )

        # --- Narrow top and bottom rails (span between stiles) ---
        door.visual(
            Box((glass_width, frame_depth, rail_height)),
            origin=Origin(xyz=(x_center, 0.0, door_height - rail_height / 2.0)),
            material=anodized_frame,
            name="top_rail",
        )
        door.visual(
            Box((glass_width, frame_depth, rail_height)),
            origin=Origin(xyz=(x_center, 0.0, rail_height / 2.0)),
            material=anodized_frame,
            name="bottom_rail",
        )

        # --- Meeting gasket (black astragal on meeting stile face) ---
        door.visual(
            Box((0.014, 0.012, door_height - 0.03)),
            origin=Origin(xyz=(direction * (door_width - 0.007), 0.028, door_height / 2.0)),
            material=shadow,
            name="meeting_gasket",
        )

        add_exit_sign(door, x_center=x_center)

        # --- Panic bar end-mount housings (bolted through glass) ---
        for idx, x in enumerate(
            (x_center - direction * 0.35, x_center + direction * 0.35)
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
            # Vertical locking rod beside the meeting stile.
            # Extended to span full door height so it contacts both latches.
            door.visual(
                Cylinder(radius=0.010, length=door_height - 0.02),
                origin=Origin(xyz=(0.805, 0.067, door_height / 2.0)),
                material=hardware,
                name="vertical_rod",
            )
            # Rod clamps contact the meeting stile front face (y = frame_depth/2 = 0.022).
            stile_front = frame_depth / 2.0
            for idx, z in enumerate((0.23, 0.78, 1.47, 1.91)):
                door.visual(
                    Box((0.070, 0.048, 0.030)),
                    origin=Origin(xyz=(0.805, stile_front + 0.024, z)),
                    material=hardware,
                    name=f"rod_clamp_{idx}",
                )
            # Top and bottom latches contact the stile front face.
            latch_y = stile_front + 0.055 / 2.0  # front face at stile_front
            door.visual(
                Box((0.040, 0.055, 0.060)),
                origin=Origin(xyz=(0.805, latch_y, door_height - 0.025)),
                material=hardware,
                name="rod_top_latch",
            )
            door.visual(
                Box((0.040, 0.055, 0.060)),
                origin=Origin(xyz=(0.805, latch_y, 0.045)),
                material=hardware,
                name="rod_bottom_latch",
            )

        return door

    door_0 = add_door_leaf("door_0", side=+1)
    door_1 = add_door_leaf("door_1", side=-1)

    # ==============================
    # Push bars (panic hardware)
    # ==============================
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

    # ==============================
    # Articulations
    # ==============================
    # Leaf hinges: closed leaves meet at center seam; positive motion opens
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    push_bar_0 = object_model.get_part("push_bar_0")
    push_bar_1 = object_model.get_part("push_bar_1")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    hinge_1 = object_model.get_articulation("frame_to_door_1")
    bar_slide_0 = object_model.get_articulation("door_0_to_push_bar_0")
    bar_slide_1 = object_model.get_articulation("door_1_to_push_bar_1")

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

    # --- Variant-specific: glazed stile-and-rail construction ---
    # Glass infill is bounded by the hinge stile in X (proves framed glass, not solid slab).
    ctx.expect_gap(
        door_0,
        door_0,
        axis="x",
        positive_elem="glass_infill",
        negative_elem="hinge_stile",
        min_gap=-0.005,
        max_gap=0.005,
        name="glass_infill is bounded by hinge_stile on door_0 (stile-and-rail frame)",
    )
    # Glass infill is bounded by the bottom rail in Z.
    ctx.expect_gap(
        door_0,
        door_0,
        axis="z",
        positive_elem="glass_infill",
        negative_elem="bottom_rail",
        min_gap=-0.005,
        max_gap=0.005,
        name="glass_infill is bounded by bottom_rail on door_0 (stile-and-rail frame)",
    )
    # Glass is fully within the stile-and-rail frame depth (proves transparent glazed panel).
    ctx.expect_within(
        door_0,
        door_0,
        axes="y",
        inner_elem="glass_infill",
        outer_elem="hinge_stile",
        margin=0.001,
        name="glass_infill is recessed within stile-and-rail frame depth on door_0",
    )

    # --- Functional checks ---
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
        elem_b="glass_infill",
        min_overlap=0.20,
        name="push_bar_0 spans across door_0 glass infill",
    )
    ctx.expect_overlap(
        push_bar_1,
        door_1,
        axes="x",
        elem_a="bar_rail",
        elem_b="glass_infill",
        min_overlap=0.20,
        name="push_bar_1 spans across door_1 glass infill",
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
    with ctx.pose({bar_slide_0: 0.015, bar_slide_1: 0.015}):
        depressed_bar = ctx.part_world_position(push_bar_0)
    ctx.check(
        "push bars depress toward the door",
        rest_bar is not None and depressed_bar is not None and depressed_bar[1] < rest_bar[1] - 0.010,
        details=f"rest={rest_bar}, depressed={depressed_bar}",
    )

    return ctx.report()


object_model = build_object_model()
