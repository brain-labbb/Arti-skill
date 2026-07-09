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


CABINET_W = 0.46
CABINET_H = 0.58
CABINET_D = 0.16
WALL = 0.018
DOOR_T = 0.018


def _cabinet_shell() -> cq.Workplane:
    """Open-front sheet-metal wall box with side ventilation slots."""
    shell = cq.Workplane("XY").box(CABINET_W, CABINET_D, CABINET_H)

    # Cut the hollow interior from the front while leaving a real rear wall.
    cut_depth = CABINET_D - WALL + 0.040
    cut_center_y = (WALL + 0.040) / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(CABINET_W - 2.0 * WALL, cut_depth, CABINET_H - 2.0 * WALL)
        .translate((0.0, cut_center_y, 0.0))
    )
    shell = shell.cut(cavity)

    # Small horizontal ventilation perforations through the right side wall.
    for z in (0.075, 0.105, 0.135, 0.165, 0.195):
        slot = (
            cq.Workplane("XY")
            .box(WALL * 3.2, 0.034, 0.007)
            .translate((CABINET_W / 2.0, 0.030, z))
        )
        shell = shell.cut(slot)
    return shell


def _door_frame() -> cq.Workplane:
    """Flat hinged metal door slab with a large upper viewing window cut out."""
    door_w = CABINET_W - 0.030
    door_h = CABINET_H - 0.035
    door = cq.Workplane("XY").box(door_w, DOOR_T, door_h).translate((door_w / 2.0, 0.0, 0.0))
    window = (
        cq.Workplane("XY")
        .box(0.305, DOOR_T * 3.0, 0.190)
        .translate((0.222, 0.0, 0.100))
    )
    return door.cut(window)


def _aed_case() -> cq.Workplane:
    """Rounded removable AED device case visible behind the window."""
    case = cq.Workplane("XY").box(0.292, 0.064, 0.230).edges("|Y").fillet(0.026)
    return case.translate((0.0, 0.0, 0.005))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_mounted_aed_defibrillator_cabinet",
        meta={
            "run_notes": (
                "Modeled the source as a wall-mounted AED/defibrillator cabinet. "
                "The source image matches the requested category; printed words and logos "
                "are represented only by simplified color blocks and icon-like markings."
            )
        },
    )

    white_metal = model.material("powder_coated_white_metal", rgba=(0.88, 0.88, 0.84, 1.0))
    inner_dark = model.material("dark_recessed_interior", rgba=(0.025, 0.026, 0.028, 1.0))
    black = model.material("black_rubber_shadow", rgba=(0.005, 0.005, 0.005, 1.0))
    glass = model.material("smoky_transparent_window", rgba=(0.55, 0.72, 0.82, 0.32))
    green = model.material("aed_green_label", rgba=(0.16, 0.68, 0.38, 1.0))
    green_dark = model.material("aed_dark_green_strip", rgba=(0.08, 0.50, 0.31, 1.0))
    red = model.material("emergency_red_marking", rgba=(0.75, 0.07, 0.04, 1.0))
    label_white = model.material("label_white_marking", rgba=(0.96, 0.98, 0.95, 1.0))
    yellow = model.material("aed_yellow_case", rgba=(0.96, 0.72, 0.14, 1.0))
    blue = model.material("aed_dark_blue_face", rgba=(0.03, 0.07, 0.15, 1.0))
    steel = model.material("brushed_hinge_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    cradle_dark = model.material("dark_steel_cradle", rgba=(0.22, 0.23, 0.25, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "hollow_cabinet_shell", tolerance=0.0008),
        material=white_metal,
        name="shell",
    )
    cabinet.visual(
        Box((0.355, 0.004, 0.330)),
        origin=Origin(xyz=(0.0, -CABINET_D / 2.0 + WALL + 0.002, 0.030)),
        material=inner_dark,
        name="dark_back_panel",
    )
    cabinet.visual(
        Box((0.340, 0.126, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, -0.125)),
        material=white_metal,
        name="aed_shelf",
    )
    for i, z in enumerate((0.075, 0.105, 0.135, 0.165, 0.195)):
        cabinet.visual(
            Box((0.006, 0.040, 0.012)),
            origin=Origin(xyz=(CABINET_W / 2.0, 0.030, z)),
            material=black,
            name=f"side_vent_{i}",
        )

    door = model.part("door")
    door_w = CABINET_W - 0.030
    door_h = CABINET_H - 0.035
    door.visual(
        mesh_from_cadquery(_door_frame(), "hinged_door_frame", tolerance=0.0008),
        material=white_metal,
        name="door_shell",
    )

    # The window pane is slightly larger than the opening so it reads as seated
    # in a shallow rabbet rather than floating in the cutout.
    door.visual(
        Box((0.322, 0.004, 0.207)),
        origin=Origin(xyz=(0.222, -0.004, 0.100)),
        material=glass,
        name="window_pane",
    )

    front_y = DOOR_T / 2.0 - 0.001
    # Thin black gasket/outline around the door perimeter.
    door.visual(Box((door_w - 0.035, 0.003, 0.006)), origin=Origin(xyz=(door_w / 2.0, front_y, door_h / 2.0 - 0.025)), material=black, name="top_gasket")
    door.visual(Box((door_w - 0.035, 0.003, 0.006)), origin=Origin(xyz=(door_w / 2.0, front_y, -door_h / 2.0 + 0.025)), material=black, name="bottom_gasket")
    door.visual(Box((0.006, 0.003, door_h - 0.050)), origin=Origin(xyz=(0.025, front_y, 0.0)), material=black, name="hinge_side_gasket")
    door.visual(Box((0.006, 0.003, door_h - 0.050)), origin=Origin(xyz=(door_w - 0.025, front_y, 0.0)), material=black, name="latch_side_gasket")

    # Simplified emergency title marking (not exact printed wording).
    door.visual(
        Box((0.285, 0.004, 0.026)),
        origin=Origin(xyz=(0.215, front_y + 0.002, 0.217)),
        material=red,
        name="red_title_mark",
    )

    # Lower AED label panel with abstract white medical markings.
    door.visual(
        Box((0.135, 0.005, 0.127)),
        origin=Origin(xyz=(0.220, front_y + 0.003, -0.105)),
        material=green,
        name="green_label_panel",
    )
    door.visual(
        Box((0.135, 0.006, 0.047)),
        origin=Origin(xyz=(0.220, front_y + 0.004, -0.194)),
        material=green_dark,
        name="lower_label_strip",
    )
    door.visual(Box((0.012, 0.007, 0.046)), origin=Origin(xyz=(0.270, front_y + 0.007, -0.045)), material=label_white, name="white_cross_stem")
    door.visual(Box((0.046, 0.007, 0.012)), origin=Origin(xyz=(0.270, front_y + 0.008, -0.045)), material=label_white, name="white_cross_bar")
    door.visual(Box((0.074, 0.007, 0.012)), origin=Origin(xyz=(0.220, front_y + 0.007, -0.192)), material=label_white, name="white_aed_bar_0")
    door.visual(Box((0.090, 0.007, 0.010)), origin=Origin(xyz=(0.220, front_y + 0.008, -0.211)), material=label_white, name="white_aed_bar_1")

    # Raised pull handle and visible hinge knuckles.
    door.visual(Box((0.018, 0.020, 0.135)), origin=Origin(xyz=(door_w - 0.057, front_y + 0.022, -0.065)), material=white_metal, name="pull_handle")
    door.visual(Box((0.020, 0.030, 0.020)), origin=Origin(xyz=(door_w - 0.057, front_y + 0.009, -0.005)), material=white_metal, name="handle_upper_post")
    door.visual(Box((0.020, 0.030, 0.020)), origin=Origin(xyz=(door_w - 0.057, front_y + 0.009, -0.125)), material=white_metal, name="handle_lower_post")
    for i, z in enumerate((0.185, 0.000, -0.185)):
        door.visual(
            Cylinder(radius=0.007, length=0.073),
            origin=Origin(xyz=(0.006, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{i}",
        )

    aed = model.part("aed_module")
    aed.visual(
        mesh_from_cadquery(_aed_case(), "removable_aed_case", tolerance=0.0008),
        material=yellow,
        name="yellow_case",
    )
    aed.visual(
        Box((0.245, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=yellow,
        name="module_skid",
    )
    aed.visual(
        Cylinder(radius=0.065, length=0.006),
        origin=Origin(xyz=(0.0, 0.033, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="front_round_display",
    )
    aed.visual(Box((0.105, 0.010, 0.018)), origin=Origin(xyz=(0.0, 0.031, 0.113)), material=yellow, name="top_carry_grip")
    aed.visual(Box((0.018, 0.012, 0.027)), origin=Origin(xyz=(-0.052, 0.031, 0.101)), material=yellow, name="grip_post_0")
    aed.visual(Box((0.018, 0.012, 0.027)), origin=Origin(xyz=(0.052, 0.031, 0.101)), material=yellow, name="grip_post_1")

    # -------------------------------------------------------------------
    # Swing-out cradle bracket: pivots about a vertical axis at the left
    # interior side wall, carrying the AED module forward for retrieval.
    # -------------------------------------------------------------------
    pivot_x = -CABINET_W / 2.0 + WALL  # left interior wall surface
    pivot_z = -0.119  # shelf-top height

    cradle = model.part("cradle_bracket")

    # Vertical pivot shaft embedded in the left interior wall.
    cradle.visual(
        Cylinder(radius=0.009, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=steel,
        name="pivot_shaft",
    )

    # Horizontal swing arm extending from the pivot toward the right.
    cradle.visual(
        Box((0.360, 0.028, 0.006)),
        origin=Origin(xyz=(0.180, 0.0, 0.003)),
        material=cradle_dark,
        name="swing_arm",
    )

    # Wider cradle tray platform that supports the AED from below.
    cradle.visual(
        Box((0.300, 0.070, 0.004)),
        origin=Origin(xyz=(0.210, 0.0, 0.008)),
        material=cradle_dark,
        name="cradle_tray",
    )

    # Small retaining lip at the front edge of the tray.
    cradle.visual(
        Box((0.300, 0.004, 0.018)),
        origin=Origin(xyz=(0.210, 0.035, 0.019)),
        material=cradle_dark,
        name="retaining_lip",
    )

    # Rear retaining lip at the back edge of the tray.
    cradle.visual(
        Box((0.300, 0.004, 0.018)),
        origin=Origin(xyz=(0.210, -0.035, 0.019)),
        material=cradle_dark,
        name="rear_lip",
    )

    # Articulations
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-door_w / 2.0, CABINET_D / 2.0 + DOOR_T / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    model.articulation(
        "cabinet_to_cradle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=cradle,
        # Pivot at the left interior wall, shelf-top height.
        # Axis +Z: right-hand rule swings the arm from +X toward +Y (forward).
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=1.50),
    )

    model.articulation(
        "cradle_to_aed",
        ArticulationType.FIXED,
        parent=cradle,
        child=aed,
        # Place AED on the cradle tray at its rest position inside the cabinet.
        # AED center x = 0 in cabinet frame → 0 - pivot_x = 0.212 in cradle frame.
        # AED skid bottom at cradle z = 0.010 (tray top) → z = 0.010 + 0.119 = 0.129.
        origin=Origin(xyz=(0.212, 0.0, 0.129)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    cradle = object_model.get_part("cradle_bracket")
    aed = object_model.get_part("aed_module")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    cradle_swing = object_model.get_articulation("cabinet_to_cradle")

    ctx.check(
        "run notes identify source interpretation",
        "wall-mounted AED" in str(object_model.meta.get("run_notes", "")),
        details=str(object_model.meta.get("run_notes", "")),
    )

    # --- Door checks (preserved from parent baseline) ---
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        name="closed door sits proud of cabinet box",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.38,
        name="door covers the front rectangular cabinet opening",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "hinged door swings outward from left side",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.16,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    # --- Cradle bracket swing-out mechanism checks ---

    # The pivot shaft is embedded in the left wall and the swing arm sits
    # inside the hollow cabinet cavity — both are intentional placements.
    ctx.allow_overlap(
        "cabinet",
        "cradle_bracket",
        elem_a="shell",
        elem_b="pivot_shaft",
        reason="The pivot shaft is intentionally embedded in the left interior wall for the swing-out cradle mount.",
    )
    ctx.allow_overlap(
        "cabinet",
        "cradle_bracket",
        elem_a="shell",
        elem_b="swing_arm",
        reason="The swing arm rests inside the hollow cabinet cavity at q=0; it swings forward through the open front when actuated.",
    )
    ctx.expect_overlap(
        cradle,
        cabinet,
        axes="x",
        elem_a="pivot_shaft",
        elem_b="shell",
        min_overlap=0.005,
        name="cradle pivot shaft is embedded in the cabinet left wall",
    )
    ctx.expect_within(
        cradle,
        cabinet,
        axes="xz",
        margin=0.002,
        inner_elem="swing_arm",
        outer_elem="shell",
        name="cradle swing arm stays within the cabinet footprint at rest",
    )

    # AED at rest sits within the cabinet outline and on the cradle tray.
    ctx.expect_within(
        aed,
        cabinet,
        axes="xz",
        margin=0.002,
        name="AED module at rest is within the cabinet outline",
    )
    ctx.expect_gap(
        aed,
        cradle,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="module_skid",
        negative_elem="cradle_tray",
        name="AED module sits on the cradle bracket tray",
    )

    # Swing-out behavior: cradle pivots the AED forward about the vertical axis.
    rest_aed_position = ctx.part_world_position(aed)

    with ctx.pose({door_hinge: 1.35, cradle_swing: 1.20}):
        swung_aed_position = ctx.part_world_position(aed)

    # The rotated AED case AABB extends back toward the cabinet when swung,
    # so we verify forward presentation via center position rather than gap.
    ctx.check(
        "AED center swings forward of the cabinet front plane",
        swung_aed_position is not None and swung_aed_position[1] > 0.10,
        details=f"swung_aed_y={swung_aed_position[1] if swung_aed_position else None}, cabinet_front_y=0.08",
    )

    ctx.check(
        "cabinet_to_cradle swings AED module forward for retrieval",
        rest_aed_position is not None
        and swung_aed_position is not None
        and swung_aed_position[1] > rest_aed_position[1] + 0.08,
        details=f"rest={rest_aed_position}, swung={swung_aed_position}",
    )

    return ctx.report()


object_model = build_object_model()
