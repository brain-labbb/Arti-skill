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
    Sphere,
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


BEACON_BODY_R = 0.034
BEACON_BODY_H = 0.032
BEACON_FLANGE_R = 0.042
BEACON_FLANGE_H = 0.006
BEACON_DOME_R = 0.030


def _beacon_housing() -> cq.Workplane:
    """Cylindrical alarm beacon base housing with mounting flange and sounder slots."""
    body = cq.Workplane("XY").cylinder(BEACON_BODY_H, BEACON_BODY_R)
    flange = (
        cq.Workplane("XY")
        .cylinder(BEACON_FLANGE_H, BEACON_FLANGE_R)
        .translate((0.0, 0.0, -(BEACON_BODY_H - BEACON_FLANGE_H) / 2.0))
    )
    housing = body.union(flange)
    # Cut horizontal sounder grille slots around the body perimeter.
    for angle in range(0, 360, 45):
        slot = (
            cq.Workplane("XY")
            .box(BEACON_BODY_R * 2.4, 0.004, 0.008)
            .translate((0.0, 0.0, 0.006))
        )
        slot = slot.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), float(angle))
        housing = housing.cut(slot)
    return housing


def _beacon_dome() -> cq.Workplane:
    """Hemispherical translucent polycarbonate dome lens."""
    sphere = cq.Workplane("XY").sphere(BEACON_DOME_R)
    cutter = (
        cq.Workplane("XY")
        .box(BEACON_DOME_R * 3.0, BEACON_DOME_R * 3.0, BEACON_DOME_R * 2.0)
        .translate((0.0, 0.0, -BEACON_DOME_R))
    )
    return sphere.cut(cutter)


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
    beacon_dark = model.material("beacon_housing_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    amber_lens = model.material("amber_translucent_lens", rgba=(0.92, 0.38, 0.05, 0.55))
    led_green = model.material("status_led_green", rgba=(0.10, 0.78, 0.22, 1.0))

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

    # Door-edge status indicator LED near the latch side upper corner,
    # seated directly against the door front face.
    door.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(door_w - 0.022, DOOR_T / 2.0 + 0.005, door_h / 2.0 - 0.028)),
        material=led_green,
        name="status_led",
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
        "cabinet_to_aed",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=aed,
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    # ── Rooftop alarm strobe beacon (fixed mounted subassembly) ──
    strobe_beacon = model.part("strobe_beacon")
    # Beacon housing sits on top of the cabinet shell, centered laterally
    # and slightly toward the rear wall for a realistic mounting position.
    strobe_beacon.visual(
        mesh_from_cadquery(_beacon_housing(), "beacon_base_housing", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, BEACON_BODY_H / 2.0)),
        material=beacon_dark,
        name="beacon_base",
    )
    strobe_beacon.visual(
        mesh_from_cadquery(_beacon_dome(), "beacon_dome_lens", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, BEACON_BODY_H - 0.003)),
        material=amber_lens,
        name="beacon_dome",
    )

    model.articulation(
        "cabinet_to_beacon",
        ArticulationType.FIXED,
        parent=cabinet,
        child=strobe_beacon,
        origin=Origin(xyz=(0.0, -0.015, CABINET_H / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    aed = object_model.get_part("aed_module")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    ctx.check(
        "run notes identify source interpretation",
        "wall-mounted AED" in str(object_model.meta.get("run_notes", "")),
        details=str(object_model.meta.get("run_notes", "")),
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

    # ── Strobe beacon and status LED checks ──
    strobe_beacon = object_model.get_part("strobe_beacon")
    beacon_joint = object_model.get_articulation("cabinet_to_beacon")

    ctx.check(
        "cabinet_to_beacon FIXED joint exists",
        beacon_joint is not None
        and beacon_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint type: {beacon_joint.articulation_type if beacon_joint else 'missing'}",
    )

    beacon_aabb = ctx.part_world_aabb(strobe_beacon)
    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check(
        "strobe_beacon sits on top of cabinet shell",
        beacon_aabb is not None
        and cabinet_aabb is not None
        and beacon_aabb[0][2] >= cabinet_aabb[1][2] - 0.005,
        details=f"beacon min z={beacon_aabb[0][2] if beacon_aabb else None}, "
        f"cabinet max z={cabinet_aabb[1][2] if cabinet_aabb else None}",
    )

    ctx.expect_within(
        strobe_beacon,
        cabinet,
        axes="xy",
        margin=0.010,
        name="strobe_beacon footprint within cabinet outline",
    )

    dome_visual = strobe_beacon.get_visual("beacon_dome")
    ctx.check(
        "beacon dome has amber translucent lens material",
        dome_visual is not None
        and dome_visual.material is not None
        and dome_visual.material.rgba is not None
        and dome_visual.material.rgba[3] < 0.90,
        details=f"material rgba: {dome_visual.material.rgba if dome_visual and dome_visual.material else 'missing'}",
    )

    led_visual = door.get_visual("status_led")
    ctx.check(
        "door has status_led indicator visual",
        led_visual is not None,
        details="status_led visual missing from door part",
    )

    return ctx.report()


object_model = build_object_model()
