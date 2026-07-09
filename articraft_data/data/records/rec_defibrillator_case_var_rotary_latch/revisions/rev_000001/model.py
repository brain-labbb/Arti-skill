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
    dark_zinc = model.material("dark_zinc_latch", rgba=(0.32, 0.33, 0.35, 1.0))

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

    # Quarter-turn cam-latch boss on the latch edge and visible hinge knuckles.
    door.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(
            xyz=(door_w - 0.057, DOOR_T / 2.0 + 0.003, -0.065),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="knob_boss",
    )
    for i, z in enumerate((0.185, 0.000, -0.185)):
        door.visual(
            Cylinder(radius=0.007, length=0.073),
            origin=Origin(xyz=(0.006, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{i}",
        )

    # Quarter-turn rotary latch knob (T-handle cam latch on the door latch edge).
    latch_knob = model.part("latch_knob")
    # Circular face plate that sits on the boss.
    latch_knob.visual(
        Cylinder(radius=0.021, length=0.007),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_zinc,
        name="knob_plate",
    )
    # T-handle wing grip that the user twists.
    latch_knob.visual(
        Box((0.060, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=dark_zinc,
        name="knob_wing",
    )
    # Small position indicator ridge on the knob face.
    latch_knob.visual(
        Box((0.003, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
        material=steel,
        name="knob_indicator",
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
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_knob,
        # Origin in door frame: latch-edge x, boss front face y, handle height z.
        origin=Origin(xyz=(door_w - 0.057, DOOR_T / 2.0 + 0.006, -0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0,
        ),
    )
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
    latch_knob = object_model.get_part("latch_knob")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    door_latch = object_model.get_articulation("door_to_latch")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    ctx.check(
        "run notes identify source interpretation",
        "wall-mounted AED" in str(object_model.meta.get("run_notes", "")),
        details=str(object_model.meta.get("run_notes", "")),
    )

    # Quarter-turn latch knob: part and joint existence plus rotation proof.
    knob_visual_names = {v.name for v in latch_knob.visuals}
    ctx.check(
        "latch_knob part has wing grip and face plate",
        "knob_wing" in knob_visual_names and "knob_plate" in knob_visual_names,
    )
    ctx.check(
        "door_to_latch is a revolute joint about the door normal",
        door_latch.articulation_type == ArticulationType.REVOLUTE
        and abs(door_latch.axis[1]) > 0.9,
    )

    # At rest (q=0) the wing is horizontal; at quarter-turn (q=pi/2) it should
    # be vertical, so its Z extent grows and X extent shrinks.
    rest_knob_aabb = ctx.part_world_aabb(latch_knob)
    with ctx.pose({door_latch: math.pi / 2.0}):
        turned_knob_aabb = ctx.part_world_aabb(latch_knob)
    ctx.check(
        "quarter-turn latch knob rotates the wing grip",
        rest_knob_aabb is not None
        and turned_knob_aabb is not None
        and (turned_knob_aabb[1][2] - turned_knob_aabb[0][2])
        > (rest_knob_aabb[1][2] - rest_knob_aabb[0][2]) + 0.010,
        details=f"rest={rest_knob_aabb}, turned={turned_knob_aabb}",
    )

    # Knob sits on the door face at the latch edge.
    ctx.expect_contact(
        latch_knob,
        door,
        elem_a="knob_plate",
        elem_b="knob_boss",
        contact_tol=0.002,
        name="latch knob plate seats against the door knob boss",
    )

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

    closed_door_aabb = ctx.part_world_aabb(door)
    rest_aed_position = ctx.part_world_position(aed)

    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "hinged door swings outward from left side",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.16,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    with ctx.pose({door_hinge: 1.35, aed_slide: 0.180}):
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
