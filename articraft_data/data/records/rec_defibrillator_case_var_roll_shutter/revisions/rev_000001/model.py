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
SLAT_T = 0.012

SLAT_COUNT = 7
SLAT_W = CABINET_W - 0.036          # 0.424 — fits inside the front opening
SLAT_H = 0.074
SLAT_GAP = 0.004
SLAT_PITCH = SLAT_H + SLAT_GAP      # 0.078
SHUTTER_TOTAL_H = SLAT_COUNT * SLAT_H + (SLAT_COUNT - 1) * SLAT_GAP  # 0.542


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


def _shutter_slat_geometry() -> cq.Workplane:
    """Single horizontal roller-shutter slat with lightly rounded long edges."""
    return (
        cq.Workplane("XY")
        .box(SLAT_W, SLAT_T, SLAT_H)
        .edges("|X")
        .fillet(0.002)
    )


def _aed_case() -> cq.Workplane:
    """Rounded removable AED device case visible behind the window."""
    case = cq.Workplane("XY").box(0.292, 0.064, 0.230).edges("|Y").fillet(0.026)
    return case.translate((0.0, 0.0, 0.005))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_mounted_aed_defibrillator_cabinet",
        meta={
            "run_notes": (
                "Roller-shutter variant of the wall-mounted AED/defibrillator cabinet. "
                "The hinged door is replaced by a segmented roll-up shutter of 7 horizontal "
                "slats on a prismatic joint that raises the shutter upward to clear the front "
                "opening. Printed words and logos are represented only by simplified color blocks."
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

    # ── Cabinet body (root) ──────────────────────────────────────────────
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

    # Vertical guide rails that channel the shutter slats
    rail_h = CABINET_H - 0.020
    rail_x_offset = SLAT_W / 2.0 + 0.008
    cabinet.visual(
        Box((0.014, 0.024, rail_h)),
        origin=Origin(xyz=(-rail_x_offset, CABINET_D / 2.0 + 0.002, 0.0)),
        material=steel,
        name="left_guide_rail",
    )
    cabinet.visual(
        Box((0.014, 0.024, rail_h)),
        origin=Origin(xyz=(rail_x_offset, CABINET_D / 2.0 + 0.002, 0.0)),
        material=steel,
        name="right_guide_rail",
    )
    # Top roller housing where the shutter retracts into
    cabinet.visual(
        Box((SLAT_W + 0.030, 0.048, 0.032)),
        origin=Origin(xyz=(0.0, CABINET_D / 2.0 - 0.008, CABINET_H / 2.0 - 0.004)),
        material=white_metal,
        name="roller_housing",
    )

    # ── Shutter (replaces hinged door) ───────────────────────────────────
    shutter = model.part("shutter")

    # Shared slat mesh reused across all indexed slats
    slat_mesh = mesh_from_cadquery(
        _shutter_slat_geometry(), "shutter_slat", tolerance=0.0008
    )
    shutter_bottom_z = -SHUTTER_TOTAL_H / 2.0 + SLAT_H / 2.0  # centre of bottom slat

    for i in range(SLAT_COUNT):
        z_offset = shutter_bottom_z + i * SLAT_PITCH
        shutter.visual(
            slat_mesh,
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=white_metal,
            name=f"shutter_slat_{i}",
        )

    # Vertical end-lock channels on the curtain edges connect all slats
    # (realistic roller-shutter construction: side channels retain the slats)
    channel_w = 0.014
    channel_h = SHUTTER_TOTAL_H + 0.004
    channel_x = SLAT_W / 2.0 - channel_w / 2.0 - 0.002
    shutter.visual(
        Box((channel_w, SLAT_T, channel_h)),
        origin=Origin(xyz=(-channel_x, 0.0, 0.0)),
        material=steel,
        name="left_endlock_channel",
    )
    shutter.visual(
        Box((channel_w, SLAT_T, channel_h)),
        origin=Origin(xyz=(channel_x, 0.0, 0.0)),
        material=steel,
        name="right_endlock_channel",
    )

    # Transparent viewing window seated into the upper slat faces
    embed_y = SLAT_T / 2.0 - 0.001  # centre slightly embedded in slat face
    window_z = shutter_bottom_z + 5 * SLAT_PITCH
    shutter.visual(
        Box((0.295, 0.005, 0.130)),
        origin=Origin(xyz=(0.0, embed_y, window_z)),
        material=glass,
        name="window_pane",
    )

    # Green emergency label panel seated into a lower-middle slat
    label_z = shutter_bottom_z + 2 * SLAT_PITCH
    shutter.visual(
        Box((0.128, 0.005, SLAT_H - 0.012)),
        origin=Origin(xyz=(0.0, embed_y, label_z)),
        material=green,
        name="green_label_panel",
    )
    shutter.visual(
        Box((0.128, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, embed_y + 0.001, label_z - SLAT_H / 2.0 + 0.012)),
        material=green_dark,
        name="lower_label_strip",
    )
    # Simplified white medical cross (mounted on the green panel)
    shutter.visual(
        Box((0.012, 0.007, 0.036)),
        origin=Origin(xyz=(0.055, embed_y + 0.004, label_z)),
        material=label_white,
        name="white_cross_stem",
    )
    shutter.visual(
        Box((0.036, 0.007, 0.012)),
        origin=Origin(xyz=(0.055, embed_y + 0.005, label_z)),
        material=label_white,
        name="white_cross_bar",
    )

    # Red title marking seated into the top slat face
    title_z = shutter_bottom_z + (SLAT_COUNT - 1) * SLAT_PITCH
    shutter.visual(
        Box((0.275, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, embed_y, title_z)),
        material=red,
        name="red_title_mark",
    )

    # Bottom lift bar (reusing pull_handle as required)
    # Embedded into the bottom slat face for connectivity
    handle_y = SLAT_T / 2.0 + 0.005
    shutter.visual(
        Box((0.100, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, handle_y, shutter_bottom_z - SLAT_H / 2.0 - 0.004)),
        material=white_metal,
        name="pull_handle",
    )

    # ── AED module ───────────────────────────────────────────────────────
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

    # ── Articulations ────────────────────────────────────────────────────
    # Prismatic shutter joint: +Z raises the shutter to clear the opening.
    model.articulation(
        "cabinet_to_shutter",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=shutter,
        origin=Origin(xyz=(0.0, CABINET_D / 2.0 + SLAT_T / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.5, lower=0.0, upper=0.560,
        ),
    )
    # AED prismatic slide (preserved from parent)
    model.articulation(
        "cabinet_to_aed",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=aed,
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0, velocity=0.35, lower=0.0, upper=0.220,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    shutter = object_model.get_part("shutter")
    aed = object_model.get_part("aed_module")
    shutter_slide = object_model.get_articulation("cabinet_to_shutter")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    # ── Variant identity ──
    ctx.check(
        "run notes identify roller-shutter variant",
        "Roller-shutter" in str(object_model.meta.get("run_notes", "")),
        details=str(object_model.meta.get("run_notes", "")),
    )

    # ── Structural topology: loop-generated slat multiplicity ──
    shutter_visual_names = [v.name for v in shutter.visuals]
    slat_names = sorted(
        n for n in shutter_visual_names if n.startswith("shutter_slat_")
    )
    ctx.check(
        "shutter built from 7 indexed slats via loop helper",
        len(slat_names) == 7,
        details=f"found: {slat_names}",
    )

    # ── Non-fixed prismatic opening joint ──
    ctx.check(
        "cabinet_to_shutter is a non-fixed prismatic joint",
        shutter_slide.motion_limits.upper > shutter_slide.motion_limits.lower,
        details=(
            f"limits: [{shutter_slide.motion_limits.lower}, "
            f"{shutter_slide.motion_limits.upper}]"
        ),
    )

    # ── Closed shutter covers the front aperture ──
    ctx.expect_overlap(
        shutter,
        cabinet,
        axes="xz",
        min_overlap=0.38,
        name="closed shutter covers the front rectangular cabinet opening",
    )

    # ── AED containment and shelf seating (preserved) ──
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

    # ── Shutter travel: raises upward to clear the aperture ──
    closed_pos = ctx.part_world_position(shutter)
    with ctx.pose({shutter_slide: 0.560}):
        open_pos = ctx.part_world_position(shutter)
    ctx.check(
        "shutter raises upward to clear the front opening",
        closed_pos is not None
        and open_pos is not None
        and open_pos[2] > closed_pos[2] + 0.50,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    # ── AED removable when shutter is fully raised ──
    rest_aed_pos = ctx.part_world_position(aed)
    with ctx.pose({shutter_slide: 0.560, aed_slide: 0.180}):
        pulled_aed_pos = ctx.part_world_position(aed)
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
        rest_aed_pos is not None
        and pulled_aed_pos is not None
        and pulled_aed_pos[1] > rest_aed_pos[1] + 0.150,
        details=f"rest={rest_aed_pos}, pulled={pulled_aed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
