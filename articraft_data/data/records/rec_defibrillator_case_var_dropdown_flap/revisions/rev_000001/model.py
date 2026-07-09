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

DOOR_W = CABINET_W - 0.030
DOOR_H = CABINET_H - 0.035


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
    """Bottom-hinged flat metal door slab with a large upper viewing window cut out.

    Part frame sits at the bottom hinge edge.  Door extends upward (+Z) and
    is centered in X so the hinge line runs symmetrically across the width.
    """
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((0.0, 0.0, DOOR_H / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(0.305, DOOR_T * 3.0, 0.190)
        .translate((0.0, 0.0, DOOR_H / 2.0 + 0.100))
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
                "Drop-front variant: the door hinges at the bottom edge and folds "
                "forward-and-down about a horizontal axis (~95 deg), matching "
                "grab-and-go defibrillator cabinet designs. Source image matches the "
                "requested category; printed words and logos are represented only by "
                "simplified color blocks and icon-like markings."
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

    # ── Cabinet body (root) ─────────────────────────────────────────────
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

    # ── Drop-down door ──────────────────────────────────────────────────
    # Part frame sits at the bottom hinge line; door extends upward (+Z).
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "hinged_door_frame", tolerance=0.0008),
        material=white_metal,
        name="door_shell",
    )

    # Window pane seated in a shallow rabbet (slightly larger than cutout).
    door.visual(
        Box((0.322, 0.004, 0.207)),
        origin=Origin(xyz=(0.0, -0.004, DOOR_H / 2.0 + 0.100)),
        material=glass,
        name="window_pane",
    )

    front_y = DOOR_T / 2.0 - 0.001

    # Thin black gasket/outline around the door perimeter.
    door.visual(
        Box((DOOR_W - 0.035, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, front_y, DOOR_H - 0.025)),
        material=black,
        name="top_gasket",
    )
    door.visual(
        Box((DOOR_W - 0.035, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, front_y, 0.025)),
        material=black,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.006, 0.003, DOOR_H - 0.050)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.025, front_y, DOOR_H / 2.0)),
        material=black,
        name="hinge_side_gasket",
    )
    door.visual(
        Box((0.006, 0.003, DOOR_H - 0.050)),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.025, front_y, DOOR_H / 2.0)),
        material=black,
        name="latch_side_gasket",
    )

    # Simplified emergency title marking (not exact printed wording).
    door.visual(
        Box((0.285, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, front_y + 0.002, DOOR_H / 2.0 + 0.217)),
        material=red,
        name="red_title_mark",
    )

    # Lower AED label panel with abstract white medical markings.
    door.visual(
        Box((0.135, 0.005, 0.127)),
        origin=Origin(xyz=(0.005, front_y + 0.003, DOOR_H / 2.0 - 0.105)),
        material=green,
        name="green_label_panel",
    )
    door.visual(
        Box((0.135, 0.006, 0.047)),
        origin=Origin(xyz=(0.005, front_y + 0.004, DOOR_H / 2.0 - 0.194)),
        material=green_dark,
        name="lower_label_strip",
    )
    door.visual(
        Box((0.012, 0.007, 0.046)),
        origin=Origin(xyz=(0.055, front_y + 0.007, DOOR_H / 2.0 - 0.045)),
        material=label_white,
        name="white_cross_stem",
    )
    door.visual(
        Box((0.046, 0.007, 0.012)),
        origin=Origin(xyz=(0.055, front_y + 0.008, DOOR_H / 2.0 - 0.045)),
        material=label_white,
        name="white_cross_bar",
    )
    door.visual(
        Box((0.074, 0.007, 0.012)),
        origin=Origin(xyz=(0.005, front_y + 0.007, DOOR_H / 2.0 - 0.192)),
        material=label_white,
        name="white_aed_bar_0",
    )
    door.visual(
        Box((0.090, 0.007, 0.010)),
        origin=Origin(xyz=(0.005, front_y + 0.008, DOOR_H / 2.0 - 0.211)),
        material=label_white,
        name="white_aed_bar_1",
    )

    # Pull handle relocated to top edge as a horizontal grab bar.
    door.visual(
        Box((0.135, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, front_y + 0.022, DOOR_H - 0.030)),
        material=white_metal,
        name="pull_handle",
    )
    door.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(-0.058, front_y + 0.009, DOOR_H - 0.030)),
        material=white_metal,
        name="handle_upper_post",
    )
    door.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(0.058, front_y + 0.009, DOOR_H - 0.030)),
        material=white_metal,
        name="handle_lower_post",
    )

    # Hinge knuckles relocated to the bottom rail, oriented along the X axis.
    for i, x in enumerate((-0.150, 0.0, 0.150)):
        door.visual(
            Cylinder(radius=0.007, length=0.073),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{i}",
        )

    # ── Removable AED module ────────────────────────────────────────────
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
    aed.visual(
        Box((0.105, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.031, 0.113)),
        material=yellow,
        name="top_carry_grip",
    )
    aed.visual(
        Box((0.018, 0.012, 0.027)),
        origin=Origin(xyz=(-0.052, 0.031, 0.101)),
        material=yellow,
        name="grip_post_0",
    )
    aed.visual(
        Box((0.018, 0.012, 0.027)),
        origin=Origin(xyz=(0.052, 0.031, 0.101)),
        material=yellow,
        name="grip_post_1",
    )

    # ── Articulations ───────────────────────────────────────────────────

    # Bottom-hinge revolute: axis (-1,0,0) so positive q drops the door
    # forward-and-down from the bottom edge.  ~95 deg travel.
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, CABINET_D / 2.0 + DOOR_T / 2.0, -DOOR_H / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.66),
    )

    # Prismatic AED shelf slide (unchanged).
    model.articulation(
        "cabinet_to_aed",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=aed,
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.220),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    aed = object_model.get_part("aed_module")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    run_notes = str(object_model.meta.get("run_notes", "")).lower()
    ctx.check(
        "run notes identify drop-front bottom-hinge variant",
        "drop-front" in run_notes and "bottom" in run_notes,
        details=str(object_model.meta.get("run_notes", "")),
    )

    # ── Closed-pose checks ──────────────────────────────────────────────
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
    ctx.expect_within(
        aed,
        cabinet,
        axes="xz",
        margin=0.002,
        name="removable AED rests within the cabinet outline",
    )
    ctx.expect_gap(
        aed,
        cabinet,
        axis="z",
        max_gap=0.002,
        max_penetration=0.00001,
        positive_elem="module_skid",
        negative_elem="aed_shelf",
        name="AED module sits on the interior shelf",
    )

    # ── Drop-front hinge behaviour ──────────────────────────────────────
    closed_door_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 1.50}):
        open_door_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "cabinet_to_door bottom hinge drops door forward and down",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10  # max Y increases (forward)
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,  # max Z drops (top swings down)
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    # ── AED removal through open front ──────────────────────────────────
    rest_aed_position = ctx.part_world_position(aed)

    with ctx.pose({door_hinge: 1.50, aed_slide: 0.180}):
        pulled_aed_position = ctx.part_world_position(aed)
        ctx.expect_gap(
            aed,
            cabinet,
            axis="y",
            min_gap=0.025,
            positive_elem="yellow_case",
            negative_elem="shell",
            name="AED module can be pulled out through the open front",
        )

    ctx.check(
        "AED module translates outward when removed",
        rest_aed_position is not None
        and pulled_aed_position is not None
        and pulled_aed_position[1] > rest_aed_position[1] + 0.150,
        details=f"rest={rest_aed_position}, pulled={pulled_aed_position}",
    )

    return ctx.report()


object_model = build_object_model()
