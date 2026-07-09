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
        name="single_emergency_exit_door_with_sidelite",
        meta={
            "reference": "source image picture/Emergency Equipment/Emergency exit door/001.png",
            "run_notes": (
                "Variant fork: single active egress leaf with fixed glazed sidelite panel. "
                "The reference shows a double-door configuration; this fork replaces the "
                "second leaf (door_1) with a static sidelite in an extended frame. The active "
                "leaf retains its hinge and panic bar."
            ),
        },
    )

    galvanized = model.material("galvanized_gray", rgba=(0.56, 0.58, 0.56, 1.0))
    darker_metal = model.material("dark_frame_gray", rgba=(0.34, 0.36, 0.35, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.03, 0.035, 0.035, 1.0))
    green = model.material("exit_sign_green", rgba=(0.0, 0.62, 0.28, 1.0))
    white = model.material("sign_white", rgba=(0.94, 0.96, 0.93, 1.0))
    hardware = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.76, 1.0))
    glass = model.material("sidelite_glass_tint", rgba=(0.72, 0.78, 0.82, 0.50))

    # ── Dimensions ──
    door_width = 0.84
    door_height = 2.03
    door_thickness = 0.045
    hinge_x = 0.86
    sill_height = 0.06
    frame_height = 2.19

    sidelite_width = 0.50
    mullion_width = 0.08
    jamb_width = 0.10
    jamb_depth = 0.11

    # ── Layout (X coordinates, left jamb is reference) ──
    # Left jamb outer: -0.99, inner: -0.89
    # Door opening:    -0.89 to -0.02  (hinge at -0.86)
    # Mullion:         -0.02 to +0.06
    # Sidelite:        +0.06 to +0.56
    # Right jamb:      +0.56 to +0.66
    left_jamb_center = -0.94
    mullion_left_x = -hinge_x + door_width          # -0.02
    mullion_center_x = mullion_left_x + mullion_width / 2.0  # +0.02
    mullion_right_x = mullion_left_x + mullion_width        # +0.06
    sidelite_center_x = mullion_right_x + sidelite_width / 2.0  # +0.31
    sidelite_right_x = mullion_right_x + sidelite_width       # +0.56
    right_jamb_center = sidelite_right_x + jamb_width / 2.0   # +0.61
    right_jamb_outer = sidelite_right_x + jamb_width           # +0.66
    left_jamb_outer = left_jamb_center - jamb_width / 2.0     # -0.99

    frame_span = right_jamb_outer - left_jamb_outer   # 1.65
    frame_center_x = (left_jamb_outer + right_jamb_outer) / 2.0  # -0.165

    # ══════════════════════════════════════════════════════════════
    # FRAME (root)
    # ══════════════════════════════════════════════════════════════
    frame = model.part("frame")

    # Left jamb
    frame.visual(
        Box((jamb_width, jamb_depth, frame_height)),
        origin=Origin(xyz=(left_jamb_center, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_0",
    )
    # Right jamb (moved inward to enclose the sidelite)
    frame.visual(
        Box((jamb_width, jamb_depth, frame_height)),
        origin=Origin(xyz=(right_jamb_center, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="jamb_1",
    )
    # Header spanning full width
    frame.visual(
        Box((frame_span, jamb_depth, 0.10)),
        origin=Origin(xyz=(frame_center_x, 0.0, 2.14)),
        material=darker_metal,
        name="header",
    )
    # Threshold spanning full width
    frame.visual(
        Box((frame_span, 0.12, sill_height)),
        origin=Origin(xyz=(frame_center_x, 0.0, sill_height / 2.0)),
        material=darker_metal,
        name="threshold",
    )
    # Hinge pivot line on left jamb only
    frame.visual(
        Cylinder(radius=0.012, length=2.02),
        origin=Origin(xyz=(-0.89, 0.040, sill_height + door_height / 2.0)),
        material=shadow,
        name="hinge_line_0",
    )

    # ── Mullion (intermediate jamb between door opening and sidelite) ──
    frame.visual(
        Box((mullion_width, jamb_depth, frame_height)),
        origin=Origin(xyz=(mullion_center_x, 0.0, frame_height / 2.0)),
        material=darker_metal,
        name="mullion",
    )

    # ── Fixed sidelite glazed panel (rigidly attached to frame) ──
    # The glass is sized to exactly contact the mullion, right stile, and
    # top/bottom stiles so it forms one connected geometry island with the
    # frame. Real glazing beads seat the glass against the perimeter members.
    glass_thickness = 0.008
    stile_depth = 0.015
    stile_face = 0.028

    # Top stile
    top_stile_z = sill_height + door_height - stile_face / 2.0  # 2.076
    top_stile_z_min = top_stile_z - stile_face / 2.0            # 2.062
    # Bottom stile
    bot_stile_z = sill_height + stile_face / 2.0                # 0.074
    bot_stile_z_max = bot_stile_z + stile_face / 2.0            # 0.088

    # Glass exactly spans between the stile contact faces and the mullion/right stile
    glass_x_min = mullion_right_x                               # 0.06
    glass_x_max = sidelite_right_x - stile_face                 # 0.532
    glass_z_min = bot_stile_z_max                               # 0.088
    glass_z_max = top_stile_z_min                               # 2.062
    glass_width = glass_x_max - glass_x_min                     # 0.472
    glass_height = glass_z_max - glass_z_min                    # 1.974
    glass_center_x = (glass_x_min + glass_x_max) / 2.0          # 0.296
    glass_center_z = (glass_z_min + glass_z_max) / 2.0          # 1.075

    frame.visual(
        Box((glass_width, glass_thickness, glass_height)),
        origin=Origin(xyz=(glass_center_x, 0.0, glass_center_z)),
        material=glass,
        name="sidelite_glass",
    )

    # Sidelite perimeter glazing stiles (thin metal strips framing the glass)
    stile_x_span = sidelite_width - 0.012  # slightly shorter than full opening
    # Top stile
    frame.visual(
        Box((stile_x_span, stile_depth, stile_face)),
        origin=Origin(xyz=(sidelite_center_x, 0.0, top_stile_z)),
        material=darker_metal,
        name="sidelite_top_stile",
    )
    # Bottom stile
    frame.visual(
        Box((stile_x_span, stile_depth, stile_face)),
        origin=Origin(xyz=(sidelite_center_x, 0.0, bot_stile_z)),
        material=darker_metal,
        name="sidelite_bottom_stile",
    )
    # Right stile (mullion serves as left frame member)
    frame.visual(
        Box((stile_face, stile_depth, glass_height)),
        origin=Origin(
            xyz=(sidelite_right_x - stile_face / 2.0, 0.0, glass_center_z)
        ),
        material=darker_metal,
        name="sidelite_right_stile",
    )

    # ══════════════════════════════════════════════════════════════
    # EXIT SIGN HELPER
    # ══════════════════════════════════════════════════════════════
    def add_exit_sign(parent_part, *, x_center: float) -> None:
        parent_part.visual(
            Box((0.34, 0.007, 0.115)),
            origin=Origin(xyz=(x_center, 0.0245, 1.28)),
            material=green,
            name="exit_sign",
        )
        parent_part.visual(
            Box((0.245, 0.004, 0.018)),
            origin=Origin(xyz=(x_center, 0.0295, 1.295)),
            material=white,
            name="sign_line",
        )
        parent_part.visual(
            Box((0.170, 0.004, 0.014)),
            origin=Origin(xyz=(x_center, 0.0295, 1.258)),
            material=white,
            name="sign_small_line",
        )

    # ══════════════════════════════════════════════════════════════
    # DOOR LEAF HELPER (single active leaf)
    # ══════════════════════════════════════════════════════════════
    def add_door_leaf(name: str, *, side: int) -> object:
        """Add a metal door leaf.

        side=+1: leaf hinged at negative X, extending toward the mullion.
        """
        door = model.part(name)
        direction = float(side)
        x_center = direction * door_width / 2.0

        # Main slab
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(x_center, 0.0, door_height / 2.0)),
            material=galvanized,
            name="door_slab",
        )
        # Raised stiles and rails on the public face
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
        # Meeting-edge gasket (faces the mullion)
        door.visual(
            Box((0.014, 0.012, door_height - 0.03)),
            origin=Origin(
                xyz=(direction * (door_width - 0.007), 0.029, door_height / 2.0)
            ),
            material=shadow,
            name="meeting_gasket",
        )

        add_exit_sign(door, x_center=x_center)

        # Push-bar mount housings (fixed to door face)
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

        # Vertical locking rod (visible feature on the active leaf)
        door.visual(
            Cylinder(radius=0.010, length=1.86),
            origin=Origin(xyz=(0.805, 0.067, door_height / 2.0)),
            material=hardware,
            name="vertical_rod",
        )
        for idx, z in enumerate((0.23, 0.78, 1.47, 1.91)):
            door.visual(
                Box((0.070, 0.048, 0.030)),
                origin=Origin(xyz=(0.805, 0.046, z)),
                material=hardware,
                name=f"rod_clamp_{idx}",
            )
        door.visual(
            Box((0.040, 0.055, 0.060)),
            origin=Origin(xyz=(0.805, 0.050, door_height - 0.025)),
            material=hardware,
            name="rod_top_latch",
        )
        door.visual(
            Box((0.040, 0.055, 0.060)),
            origin=Origin(xyz=(0.805, 0.050, 0.045)),
            material=hardware,
            name="rod_bottom_latch",
        )

        return door

    # Only one active leaf (left side)
    door_0 = add_door_leaf("door_0", side=+1)

    # ══════════════════════════════════════════════════════════════
    # PUSH BAR
    # ══════════════════════════════════════════════════════════════
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

    # ══════════════════════════════════════════════════════════════
    # ARTICULATIONS
    # ══════════════════════════════════════════════════════════════

    # Door hinge: closed leaf meets the mullion; positive q swings forward (+Y).
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

    # ── Structural assertions for the sidelite variant ──

    # Active leaf hinge is present and correct
    ctx.check(
        "single active hinged leaf",
        hinge_0.child == "door_0" and hinge_0.mimic is None,
        details="door_0 should have its own non-mimicked revolute hinge.",
    )

    # Push bar is a separate child of the active leaf
    ctx.check(
        "push bar on active leaf",
        bar_slide_0.child == "push_bar_0",
        details="Panic bar should be a separate prismatic child of door_0.",
    )

    # No second hinged leaf — sidelite is fixed (TARGET change proof)
    art_names = [a.name for a in object_model.articulations]
    ctx.check(
        "sidelite has no hinge — no frame_to_door_1 articulation",
        "frame_to_door_1" not in art_names,
        details=(
            "The second leaf was replaced by a fixed glazed sidelite rigidly "
            "attached to the frame. No frame_to_door_1 joint should exist."
        ),
    )

    # Sidelite glass visual is present on the frame
    frame_visual_names = [v.name for v in frame.visuals]
    ctx.check(
        "sidelite glazed panel present on frame",
        "sidelite_glass" in frame_visual_names,
        details="Frame must carry a sidelite_glass visual for the fixed glazed panel.",
    )

    # Mullion is present between the active leaf and sidelite
    ctx.check(
        "intermediate mullion jamb present on frame",
        "mullion" in frame_visual_names,
        details="Frame must include a mullion visual separating door_0 from the sidelite.",
    )

    # ── Spatial checks ──

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

    # Sidelite glass X span is within the header (frame) X span
    ctx.expect_within(
        frame,
        frame,
        axes="x",
        margin=0.01,
        inner_elem="sidelite_glass",
        outer_elem="header",
        name="sidelite glass X is within the header span",
    )

    # ── Pose checks ──

    # Door swing
    rest_0 = ctx.part_world_aabb(door_0)
    with ctx.pose({hinge_0: 1.05}):
        opened_0 = ctx.part_world_aabb(door_0)
    ctx.check(
        "door_0 swings forward",
        rest_0 is not None
        and opened_0 is not None
        and opened_0[1][1] > rest_0[1][1] + 0.45,
        details=f"rest={rest_0}, opened={opened_0}",
    )

    # Push bar depression
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
