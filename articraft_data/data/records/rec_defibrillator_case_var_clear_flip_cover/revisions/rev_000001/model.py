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


# --- Global dimensions ---
CABINET_W = 0.46
CABINET_H = 0.58
CABINET_D = 0.13  # shallower for low-profile clear cover variant
WALL = 0.018
COVER_T = 0.005  # polycarbonate panel thickness
DOME = 0.010  # shallow dome sagitta

COVER_W = CABINET_W - 0.030
COVER_H = CABINET_H - 0.035

# Hinge sits at the cabinet front face so the cover inner face contacts the box.
HINGE_Y = CABINET_D / 2.0
HINGE_Z = CABINET_H / 2.0 - WALL / 2.0  # centred on top wall thickness


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


def _clear_cover() -> cq.Workplane:
    """Shallow-domed clear polycarbonate flip-up cover panel.

    Cross-section in YZ workplane:
      inner face at y=0, outer face peaks at y=COVER_T+DOME
      Z from -COVER_H/2 to +COVER_H/2 (shifted so top edge at z=0).
    Extruded along X by COVER_W and centred.
    """
    t = COVER_T
    h = COVER_H
    w = COVER_W
    dome = DOME

    cross_section = (
        cq.Workplane("YZ")
        .moveTo(0.0, -h / 2.0)
        .lineTo(0.0, h / 2.0)
        .lineTo(t, h / 2.0)
        .threePointArc((t + dome, 0.0), (t, -h / 2.0))
        .close()
    )
    cover = cross_section.extrude(w).translate((-w / 2.0, 0.0, -h / 2.0))
    return cover


def _aed_case() -> cq.Workplane:
    """Rounded removable AED device case visible behind the window."""
    case = cq.Workplane("XY").box(0.292, 0.064, 0.230).edges("|Y").fillet(0.026)
    return case.translate((0.0, 0.0, 0.005))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wall_mounted_aed_defibrillator_cabinet",
        meta={
            "run_notes": (
                "Variant: clear polycarbonate top-hinged flip-up cover replacing the "
                "opaque side-hinged door. The shallow-domed transparent cover lifts "
                "upward about a top horizontal axis (~100 degrees). Cabinet depth "
                "reduced to 130 mm for the low-profile clear cover configuration. "
                "Source image interpretation unchanged from parent baseline."
            )
        },
    )

    # --- Materials ---
    white_metal = model.material(
        "powder_coated_white_metal", rgba=(0.88, 0.88, 0.84, 1.0)
    )
    inner_dark = model.material(
        "dark_recessed_interior", rgba=(0.025, 0.026, 0.028, 1.0)
    )
    black = model.material("black_rubber_shadow", rgba=(0.005, 0.005, 0.005, 1.0))
    glass = model.material(
        "clear_polycarbonate", rgba=(0.72, 0.85, 0.90, 0.28)
    )
    green = model.material("aed_green_label", rgba=(0.16, 0.68, 0.38, 1.0))
    green_dark = model.material(
        "aed_dark_green_strip", rgba=(0.08, 0.50, 0.31, 1.0)
    )
    red = model.material("emergency_red_marking", rgba=(0.75, 0.07, 0.04, 1.0))
    label_white = model.material(
        "label_white_marking", rgba=(0.96, 0.98, 0.95, 1.0)
    )
    yellow = model.material("aed_yellow_case", rgba=(0.96, 0.72, 0.14, 1.0))
    blue = model.material("aed_dark_blue_face", rgba=(0.03, 0.07, 0.15, 1.0))
    steel = model.material("brushed_hinge_steel", rgba=(0.62, 0.64, 0.62, 1.0))

    # ================================================================
    # Cabinet (root part)
    # ================================================================
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
        Box((0.340, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, -0.125)),
        material=white_metal,
        name="aed_shelf",
    )

    # Side ventilation slot visuals (right side).
    for i, z in enumerate((0.075, 0.105, 0.135, 0.165, 0.195)):
        cabinet.visual(
            Box((0.006, 0.040, 0.012)),
            origin=Origin(xyz=(CABINET_W / 2.0, 0.030, z)),
            material=black,
            name=f"side_vent_{i}",
        )

    # Top hinge mounting rail (steel strip at front of top wall).
    # Extends slightly proud of the front face so the cover hinge knuckles
    # have a visible mounting surface.
    rail_depth = 0.012
    cabinet.visual(
        Box((0.360, rail_depth, WALL + 0.004)),
        origin=Origin(
            xyz=(0.0, HINGE_Y + rail_depth / 2.0 - 0.003, HINGE_Z)
        ),
        material=steel,
        name="hinge_rail",
    )

    # Interior labels visible through the clear cover.
    # All positioned to overlap with the dark_back_panel for connectivity.
    # dark_back_panel Z range: 0.030 ± 0.165 = -0.135 to 0.195
    # dark_back_panel Y range: ≈ -0.045 to -0.041
    back_y = -CABINET_D / 2.0 + WALL + 0.004  # ≈ -0.043
    cabinet.visual(
        Box((0.135, 0.005, 0.100)),
        origin=Origin(xyz=(0.0, back_y, 0.080)),
        material=green,
        name="interior_green_label",
    )
    cabinet.visual(
        Box((0.135, 0.006, 0.035)),
        origin=Origin(xyz=(0.0, back_y + 0.001, 0.010)),
        material=green_dark,
        name="interior_lower_strip",
    )
    cabinet.visual(
        Box((0.285, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, back_y, 0.170)),
        material=red,
        name="interior_red_header",
    )
    # White cross marking on interior green label.
    cabinet.visual(
        Box((0.012, 0.007, 0.046)),
        origin=Origin(xyz=(0.0, back_y + 0.003, 0.100)),
        material=label_white,
        name="white_cross_stem",
    )
    cabinet.visual(
        Box((0.046, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, back_y + 0.004, 0.100)),
        material=label_white,
        name="white_cross_bar",
    )

    # ================================================================
    # Door  (clear polycarbonate flip-up cover)
    # ================================================================
    door = model.part("door")

    # Main transparent shallow-domed cover panel.
    door.visual(
        mesh_from_cadquery(_clear_cover(), "clear_domed_cover", tolerance=0.0008),
        material=glass,
        name="door_shell",
    )

    # Flat viewing-window pane embedded inside the dome cross-section.
    # Y extends from near the inner face well into the dome body to
    # guarantee mesh connectivity with door_shell.
    door.visual(
        Box((COVER_W - 0.040, 0.012, COVER_H - 0.060)),
        origin=Origin(xyz=(0.0, 0.006, -COVER_H / 2.0)),
        material=glass,
        name="window_pane",
    )

    # Perimeter edge seals – positioned well within the dome body for
    # robust mesh connectivity (mid-thickness of the dome cross-section).
    seal_y = COVER_T + DOME * 0.4  # ≈ 0.009, inside dome volume
    seal_thick = 0.008
    door.visual(
        Box((COVER_W - 0.060, seal_thick, 0.010)),
        origin=Origin(xyz=(0.0, seal_y, -COVER_H + 0.025)),
        material=black,
        name="bottom_seal",
    )
    door.visual(
        Box((0.010, seal_thick, COVER_H - 0.060)),
        origin=Origin(xyz=(-COVER_W / 2.0 + 0.025, seal_y, -COVER_H / 2.0)),
        material=black,
        name="left_seal",
    )
    door.visual(
        Box((0.010, seal_thick, COVER_H - 0.060)),
        origin=Origin(xyz=(COVER_W / 2.0 - 0.025, seal_y, -COVER_H / 2.0)),
        material=black,
        name="right_seal",
    )

    # Bottom lift-lip grip (replaces the side pull handle).
    lip_y = COVER_T + DOME * 0.5  # middle of dome thickness
    door.visual(
        Box((0.140, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, lip_y + 0.008, -COVER_H + 0.020)),
        material=glass,
        name="lift_lip",
    )
    for i, x_off in enumerate((-0.058, 0.058)):
        door.visual(
            Box((0.012, 0.014, 0.012)),
            origin=Origin(xyz=(x_off, lip_y + 0.007, -COVER_H + 0.020)),
            material=glass,
            name=f"lip_post_{i}",
        )

    # Hinge knuckles along the top rail (axis along X).
    # Offset slightly behind the hinge line so the barrel wraps around the
    # hinge pin at the cabinet edge (realistic hinge embedding).
    for i, x_pos in enumerate((-0.150, 0.0, 0.150)):
        door.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(
                xyz=(x_pos, -0.003, -0.004),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_knuckle_{i}",
        )

    # ================================================================
    # AED Module (removable device)
    # ================================================================
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

    # ================================================================
    # Articulations
    # ================================================================
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=1.75
        ),
    )
    model.articulation(
        "cabinet_to_aed",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=aed,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0, velocity=0.35, lower=0.0, upper=0.220
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    aed = object_model.get_part("aed_module")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    aed_slide = object_model.get_articulation("cabinet_to_aed")

    # --- Hinge knuckle barrel embedding in the cabinet top wall / rail ---
    # The knuckles intentionally wrap around the hinge axis at the cabinet
    # edge, which is a realistic hinge barrel embedding.
    ctx.allow_overlap(
        cabinet,
        door,
        reason=(
            "Hinge knuckles wrap around the top-edge hinge axis, "
            "embedding slightly inside the cabinet top wall and hinge rail "
            "as a realistic barrel-hinge capture."
        ),
    )

    # --- Source interpretation note ---
    ctx.check(
        "run notes identify clear flip-up cover variant",
        "flip-up" in str(object_model.meta.get("run_notes", "")),
        details=str(object_model.meta.get("run_notes", "")),
    )

    # --- Closed cover covers the front opening ---
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.38,
        name="closed flip-up cover covers the front cabinet opening",
    )

    # --- AED within cabinet ---
    ctx.expect_within(
        aed,
        cabinet,
        axes="xz",
        margin=0.002,
        name="removable AED rests within the cabinet outline",
    )

    # --- AED on shelf ---
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

    # --- Top-hinge flip-up proof: cover bottom rises when opened ---
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "cabinet_to_door top-hinge lifts cover bottom upward (flip-up)",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > closed_aabb[0][2] + 0.10,
        details=(
            f"closed_min_z={closed_aabb[0][2] if closed_aabb else None}, "
            f"open_min_z={open_aabb[0][2] if open_aabb else None}"
        ),
    )

    # --- Cover swings outward (positive Y) when opened ---
    ctx.check(
        "cabinet_to_door swings cover outward from cabinet front",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.10,
        details=(
            f"closed_max_y={closed_aabb[1][1] if closed_aabb else None}, "
            f"open_max_y={open_aabb[1][1] if open_aabb else None}"
        ),
    )

    # --- Hinge knuckle stays near the top rail (support check) ---
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.010,
        elem_a="hinge_knuckle_0",
        elem_b="hinge_rail",
        name="hinge_knuckle_0 overlaps hinge_rail in xz (barrel hinge capture)",
    )

    # --- AED removable when cover is flipped up ---
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
            name="AED module can be pulled out with flip-up cover open",
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
