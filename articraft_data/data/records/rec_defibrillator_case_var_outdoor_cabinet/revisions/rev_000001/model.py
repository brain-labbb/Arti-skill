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

# Weatherproof roof cap
ROOF_OVERHANG_SIDE = 0.020
ROOF_OVERHANG_FRONT = 0.035
ROOF_OVERHANG_BACK = 0.012
ROOF_BACK_H = 0.045
ROOF_FRONT_H = 0.015

# Door weather rebate
REBATE_DEPTH = 0.008
REBATE_WIDTH = 0.010

# Shared vent positions (used for both shell cuts and hood visuals)
VENT_Z_POSITIONS = (0.075, 0.105, 0.135, 0.165, 0.195)

# Door dimensions (slightly reduced height for rebate swing clearance)
DOOR_W = CABINET_W - 0.030
DOOR_H = CABINET_H - 0.040


def _cabinet_shell() -> cq.Workplane:
    """Open-front sheet-metal wall box with hooded ventilation slots."""
    shell = cq.Workplane("XY").box(CABINET_W, CABINET_D, CABINET_H)

    # Hollow interior cut from the front, leaving real rear wall.
    cut_depth = CABINET_D - WALL + 0.040
    cut_center_y = (WALL + 0.040) / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(CABINET_W - 2.0 * WALL, cut_depth, CABINET_H - 2.0 * WALL)
        .translate((0.0, cut_center_y, 0.0))
    )
    shell = shell.cut(cavity)

    # Louver-style ventilation slots on the right side wall.
    for z in VENT_Z_POSITIONS:
        slot = (
            cq.Workplane("XY")
            .box(WALL * 3.2, 0.034, 0.007)
            .translate((CABINET_W / 2.0, 0.030, z))
        )
        shell = shell.cut(slot)
    return shell


def _roof_cap() -> cq.Workplane:
    """Sloped rain-shedding roof cap with front overhang and drip lip."""
    roof_w = CABINET_W + 2.0 * ROOF_OVERHANG_SIDE
    roof_d = CABINET_D + ROOF_OVERHANG_FRONT + ROOF_OVERHANG_BACK

    y_back = -(roof_d / 2.0)
    y_front = roof_d / 2.0

    # Trapezoidal cross-section in YZ plane, extruded along X.
    # High at back (rain shed toward front).
    wedge = (
        cq.Workplane("YZ")
        .moveTo(y_back, 0.0)
        .lineTo(y_front, 0.0)
        .lineTo(y_front, ROOF_FRONT_H)
        .lineTo(y_back, ROOF_BACK_H)
        .close()
        .extrude(roof_w / 2.0, both=True)
    )

    # Front drip edge flange hanging below the roof overhang.
    drip = (
        cq.Workplane("XY")
        .box(roof_w, 0.004, 0.014)
        .translate((0.0, y_front - 0.002, -0.007))
    )
    wedge = wedge.union(drip)

    # Side drip lips (left and right) for complete weather seal.
    for sign in (-1.0, 1.0):
        side_drip = (
            cq.Workplane("XY")
            .box(0.004, roof_d - 0.010, 0.010)
            .translate((sign * (roof_w / 2.0 - 0.002), 0.005, -0.005))
        )
        wedge = wedge.union(side_drip)

    return wedge


def _door_with_rebate() -> cq.Workplane:
    """Hinged door slab with weather-seal rebate lip and viewing window."""
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((DOOR_W / 2.0, 0.0, 0.0))
    )

    # Window cutout
    window = (
        cq.Workplane("XY")
        .box(0.305, DOOR_T * 3.0, 0.190)
        .translate((0.222, 0.0, 0.100))
    )
    door = door.cut(window)

    # Weather-seal rebate lip: stepped frame on the inner face that tucks
    # behind the cabinet front edges when closed.
    inner_y = -DOOR_T / 2.0

    # Top rebate
    door = door.union(
        cq.Workplane("XY")
        .box(DOOR_W, REBATE_DEPTH, REBATE_WIDTH)
        .translate(
            (DOOR_W / 2.0, inner_y - REBATE_DEPTH / 2.0, DOOR_H / 2.0 - REBATE_WIDTH / 2.0)
        )
    )
    # Bottom rebate
    door = door.union(
        cq.Workplane("XY")
        .box(DOOR_W, REBATE_DEPTH, REBATE_WIDTH)
        .translate(
            (DOOR_W / 2.0, inner_y - REBATE_DEPTH / 2.0, -DOOR_H / 2.0 + REBATE_WIDTH / 2.0)
        )
    )
    # Hinge-side rebate (left)
    door = door.union(
        cq.Workplane("XY")
        .box(REBATE_WIDTH, REBATE_DEPTH, DOOR_H - 2.0 * REBATE_WIDTH)
        .translate(
            (REBATE_WIDTH / 2.0, inner_y - REBATE_DEPTH / 2.0, 0.0)
        )
    )
    # Latch-side rebate (right)
    door = door.union(
        cq.Workplane("XY")
        .box(REBATE_WIDTH, REBATE_DEPTH, DOOR_H - 2.0 * REBATE_WIDTH)
        .translate(
            (DOOR_W - REBATE_WIDTH / 2.0, inner_y - REBATE_DEPTH / 2.0, 0.0)
        )
    )
    return door


def _aed_case() -> cq.Workplane:
    """Rounded removable AED device case visible behind the window."""
    case = cq.Workplane("XY").box(0.292, 0.064, 0.230).edges("|Y").fillet(0.026)
    return case.translate((0.0, 0.0, 0.005))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="outdoor_weatherproof_aed_cabinet",
        meta={
            "run_notes": (
                "Outdoor IP-rated weatherproof AED defibrillator cabinet variant. "
                "Changed from flat-top indoor cabinet to emergency-green outdoor "
                "enclosure with sloped rain-shedding roof cap (front overhang and "
                "drip edge), deeper door weather rebate, and hooded ventilation "
                "louvers. White AED cross signage on the door. Retains hinged front "
                "door (cabinet_to_door revolute) and removable AED module "
                "(cabinet_to_aed prismatic) from the parent baseline."
            )
        },
    )

    # --- Materials: emergency-green weatherproof body ---
    green_body = model.material("emergency_green_body", rgba=(0.12, 0.42, 0.22, 1.0))
    green_dark = model.material("dark_green_roof", rgba=(0.08, 0.30, 0.16, 1.0))
    inner_dark = model.material("dark_recessed_interior", rgba=(0.025, 0.026, 0.028, 1.0))
    black = model.material("black_rubber_shadow", rgba=(0.005, 0.005, 0.005, 1.0))
    glass = model.material("smoky_transparent_window", rgba=(0.55, 0.72, 0.82, 0.32))
    label_green = model.material("aed_green_label", rgba=(0.16, 0.68, 0.38, 1.0))
    label_green_dark = model.material("aed_dark_green_strip", rgba=(0.08, 0.50, 0.31, 1.0))
    red = model.material("emergency_red_marking", rgba=(0.75, 0.07, 0.04, 1.0))
    label_white = model.material("label_white_marking", rgba=(0.96, 0.98, 0.95, 1.0))
    yellow = model.material("aed_yellow_case", rgba=(0.96, 0.72, 0.14, 1.0))
    blue = model.material("aed_dark_blue_face", rgba=(0.03, 0.07, 0.15, 1.0))
    steel = model.material("brushed_hinge_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    gasket = model.material("rubber_weather_seal", rgba=(0.02, 0.02, 0.02, 1.0))

    # === Cabinet body (root part) ===
    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "hollow_cabinet_shell", tolerance=0.0008),
        material=green_body,
        name="shell",
    )

    # Sloped roof cap sitting proud of shell top face
    roof_y_offset = (ROOF_OVERHANG_FRONT - ROOF_OVERHANG_BACK) / 2.0
    cabinet.visual(
        mesh_from_cadquery(_roof_cap(), "sloped_roof_cap", tolerance=0.0008),
        origin=Origin(xyz=(0.0, roof_y_offset, CABINET_H / 2.0)),
        material=green_dark,
        name="roof_cap",
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
        material=green_body,
        name="aed_shelf",
    )

    # Hooded ventilation louvers on right side (loop-generated)
    for i, z in enumerate(VENT_Z_POSITIONS):
        # Dark interior behind each vent slot
        cabinet.visual(
            Box((0.006, 0.040, 0.012)),
            origin=Origin(xyz=(CABINET_W / 2.0, 0.030, z)),
            material=black,
            name=f"side_vent_{i}",
        )
        # Hood louver plate protruding outward above each slot
        cabinet.visual(
            Box((0.012, 0.038, 0.003)),
            origin=Origin(xyz=(CABINET_W / 2.0 + 0.006, 0.030, z + 0.005)),
            material=green_body,
            name=f"vent_hood_{i}",
        )

    # === Hinged door with weather rebate ===
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_with_rebate(), "hinged_door_with_rebate", tolerance=0.0008),
        material=green_body,
        name="door_shell",
    )

    # Window pane seated in door rabbet
    door.visual(
        Box((0.322, 0.004, 0.207)),
        origin=Origin(xyz=(0.222, -0.004, 0.100)),
        material=glass,
        name="window_pane",
    )

    front_y = DOOR_T / 2.0 - 0.001
    # Black rubber weather-seal gasket around door perimeter
    door.visual(
        Box((DOOR_W - 0.035, 0.003, 0.006)),
        origin=Origin(xyz=(DOOR_W / 2.0, front_y, DOOR_H / 2.0 - 0.020)),
        material=gasket,
        name="top_gasket",
    )
    door.visual(
        Box((DOOR_W - 0.035, 0.003, 0.006)),
        origin=Origin(xyz=(DOOR_W / 2.0, front_y, -DOOR_H / 2.0 + 0.020)),
        material=gasket,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.006, 0.003, DOOR_H - 0.040)),
        origin=Origin(xyz=(0.025, front_y, 0.0)),
        material=gasket,
        name="hinge_side_gasket",
    )
    door.visual(
        Box((0.006, 0.003, DOOR_H - 0.040)),
        origin=Origin(xyz=(DOOR_W - 0.025, front_y, 0.0)),
        material=gasket,
        name="latch_side_gasket",
    )

    # Emergency title marking (red banner above window)
    door.visual(
        Box((0.285, 0.004, 0.026)),
        origin=Origin(xyz=(0.215, front_y + 0.002, 0.217)),
        material=red,
        name="red_title_mark",
    )

    # White AED cross signage (companion variation ⑥)
    door.visual(
        Box((0.135, 0.005, 0.127)),
        origin=Origin(xyz=(0.220, front_y + 0.003, -0.105)),
        material=label_green,
        name="green_label_panel",
    )
    door.visual(
        Box((0.135, 0.006, 0.047)),
        origin=Origin(xyz=(0.220, front_y + 0.004, -0.194)),
        material=label_green_dark,
        name="lower_label_strip",
    )
    # Prominent white medical cross
    door.visual(
        Box((0.014, 0.007, 0.065)),
        origin=Origin(xyz=(0.270, front_y + 0.007, -0.058)),
        material=label_white,
        name="white_cross_stem",
    )
    door.visual(
        Box((0.065, 0.007, 0.014)),
        origin=Origin(xyz=(0.270, front_y + 0.008, -0.058)),
        material=label_white,
        name="white_cross_bar",
    )
    door.visual(
        Box((0.074, 0.007, 0.012)),
        origin=Origin(xyz=(0.220, front_y + 0.007, -0.192)),
        material=label_white,
        name="white_aed_bar_0",
    )
    door.visual(
        Box((0.090, 0.007, 0.010)),
        origin=Origin(xyz=(0.220, front_y + 0.008, -0.211)),
        material=label_white,
        name="white_aed_bar_1",
    )

    # Raised pull handle
    door.visual(
        Box((0.018, 0.020, 0.135)),
        origin=Origin(xyz=(DOOR_W - 0.057, front_y + 0.022, -0.065)),
        material=green_body,
        name="pull_handle",
    )
    door.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(DOOR_W - 0.057, front_y + 0.009, -0.005)),
        material=green_body,
        name="handle_upper_post",
    )
    door.visual(
        Box((0.020, 0.030, 0.020)),
        origin=Origin(xyz=(DOOR_W - 0.057, front_y + 0.009, -0.125)),
        material=green_body,
        name="handle_lower_post",
    )

    # Visible hinge knuckles
    for i, z in enumerate((0.185, 0.000, -0.185)):
        door.visual(
            Cylinder(radius=0.007, length=0.073),
            origin=Origin(xyz=(0.006, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"hinge_knuckle_{i}",
        )

    # === Removable AED module ===
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

    # === Articulations ===
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-DOOR_W / 2.0, CABINET_D / 2.0 + DOOR_T / 2.0, 0.0)),
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
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    aed = object_model.get_part("aed_module")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    # --- Variant identity ---
    ctx.check(
        "run notes identify outdoor weatherproof variant with roof cap",
        "outdoor" in str(object_model.meta.get("run_notes", "")).lower()
        and "roof" in str(object_model.meta.get("run_notes", "")).lower(),
        details=str(object_model.meta.get("run_notes", "")),
    )

    # --- Roof cap sits above shell top with drip edge overhang ---
    # The main roof body sits on top of the shell; the front drip lip
    # intentionally extends ~14mm below the shell top at the overhang.
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_penetration=0.016,
        positive_elem="roof_cap",
        negative_elem="shell",
        name="roof cap drip edge hangs at most 16mm below shell top",
    )
    ctx.expect_overlap(
        cabinet,
        cabinet,
        axes="z",
        min_overlap=0.010,
        elem_a="roof_cap",
        elem_b="shell",
        name="roof cap extends above the shell top face",
    )
    ctx.expect_overlap(
        cabinet,
        cabinet,
        axes="xy",
        min_overlap=0.15,
        elem_a="roof_cap",
        elem_b="shell",
        name="roof cap covers the shell footprint with overhang",
    )

    # --- Door rebate seats into cabinet front opening ---
    # The rebate lip intentionally tucks behind the cabinet front edges
    # (small local overlap for weather sealing).
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a="door_shell",
        elem_b="shell",
        reason=(
            "The door weather rebate lip tucks behind the cabinet front edges "
            "to create a sealed overlap, representing a real IP-rated weather seal."
        ),
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.38,
        name="rebated door covers the front cabinet opening",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a="door_shell",
        elem_b="shell",
        contact_tol=0.002,
        name="door rebate seats in contact with cabinet front edges",
    )

    # --- AED module containment and seating ---
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

    # --- Articulated door swing ---
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

    # --- AED module removal through open front ---
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
