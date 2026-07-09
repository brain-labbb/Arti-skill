from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.56
DEPTH = 0.22
HEIGHT = 0.29
FRONT_Y = -DEPTH / 2.0


def _rounded_cabinet() -> cq.Workplane:
    """A one-piece softly radiused horizontal wooden tabletop radio cabinet."""
    body = (
        cq.Workplane("XY")
        .box(WIDTH, DEPTH, HEIGHT)
        .edges()
        .fillet(0.026)
    )
    return body


def _front_cylinder_origin(x: float, y: float, z: float) -> Origin:
    # URDF/CQ cylinders are along local +Z; rotate them so the axis points
    # through the radio front.
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _add_front_bar(part, material: Material, name: str, x: float, z: float, sx: float, sz: float, *, y: float, depth: float) -> None:
    part.visual(
        Box((sx, depth, sz)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_retro_tabletop_radio")

    cherry = model.material("glossy_cherry_wood", rgba=(0.62, 0.25, 0.12, 1.0))
    cherry_dark = model.material("dark_cherry_grain", rgba=(0.36, 0.13, 0.055, 1.0))
    cherry_light = model.material("golden_wood_strip", rgba=(0.62, 0.37, 0.16, 1.0))
    black = model.material("black_lacquer_frame", rgba=(0.006, 0.006, 0.005, 1.0))
    shadow = model.material("deep_speaker_shadow", rgba=(0.015, 0.012, 0.010, 1.0))
    cream = model.material("aged_cream_cloth", rgba=(0.83, 0.78, 0.59, 1.0))
    cream_high = model.material("raised_cream_ribs", rgba=(0.93, 0.88, 0.67, 1.0))
    dial_face = model.material("warm_ivory_dial", rgba=(0.89, 0.80, 0.58, 1.0))
    dial_ink = model.material("black_frequency_ink", rgba=(0.02, 0.018, 0.014, 1.0))
    red_ink = model.material("red_frequency_ink", rgba=(0.78, 0.055, 0.035, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.86, 0.82, 1.0))
    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_rounded_cabinet(), "rounded_cherry_cabinet", tolerance=0.0015),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0)),
        material=cherry,
        name="rounded_cherry_cabinet",
    )

    # Subtle veneer grain strips on the glossy top and front lower wood band.
    for i, x in enumerate((-0.235, -0.205, -0.165, -0.125, -0.075, -0.028, 0.018, 0.064, 0.113, 0.158, 0.205, 0.242)):
        mat = cherry_dark if i % 3 != 1 else cherry_light
        cabinet.visual(
            Box((0.006 + 0.002 * (i % 2), DEPTH * 0.72, 0.0012)),
            origin=Origin(xyz=(x, -0.005 + 0.006 * ((i % 4) - 1.5), HEIGHT + 0.0001)),
            material=mat,
            name=f"top_grain_{i}",
        )

    # A dark recessed field under the front trim gives the face its set-back
    # shadow while still physically overlapping the wooden cabinet.
    cabinet.visual(
        Box((0.505, 0.019, 0.235)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0035, 0.150)),
        material=shadow,
        name="front_shadow_recess",
    )

    black_bezel = BezelGeometry(
        (0.452, 0.184),
        (0.506, 0.236),
        0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.030,
        center=False,
    )
    cabinet.visual(
        mesh_from_geometry(black_bezel, "black_front_bezel"),
        # local XY -> world XZ, local +Z depth -> world -Y
        origin=Origin(xyz=(0.0, FRONT_Y + 0.004, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="black_front_bezel",
    )

    # Warm lower control strip inside the black frame.
    cabinet.visual(
        Box((0.430, 0.024, 0.062)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.014, 0.073)),
        material=cherry_light,
        name="lower_control_strip",
    )
    for i, x in enumerate((-0.188, -0.135, -0.082, -0.030, 0.026, 0.082, 0.139, 0.190)):
        cabinet.visual(
            Box((0.004, 0.002, 0.054)),
            origin=Origin(xyz=(x, FRONT_Y - 0.027, 0.073)),
            material=cherry_dark,
            name=f"lower_grain_{i}",
        )

    # Circular speaker cone shadow behind the ribbed cream grille.
    cabinet.visual(
        Cylinder(radius=0.070, length=0.007),
        origin=_front_cylinder_origin(-0.112, FRONT_Y - 0.014, 0.176),
        material=shadow,
        name="round_speaker_shadow",
    )

    # Ribbed cream cloth grille: side rails and many connected horizontal ribs.
    grille_w = 0.424
    grille_h = 0.132
    grille_cz = 0.176
    grille_y = FRONT_Y - 0.019
    rail_depth = 0.012
    cabinet.visual(
        Box((grille_w, rail_depth, 0.010)),
        origin=Origin(xyz=(0.0, grille_y, grille_cz + grille_h / 2.0 - 0.005)),
        material=cream,
        name="grille_top_rail",
    )
    cabinet.visual(
        Box((grille_w, rail_depth, 0.010)),
        origin=Origin(xyz=(0.0, grille_y, grille_cz - grille_h / 2.0 + 0.005)),
        material=cream,
        name="grille_bottom_rail",
    )
    for side, x in enumerate((-grille_w / 2.0 + 0.006, grille_w / 2.0 - 0.006)):
        cabinet.visual(
            Box((0.012, rail_depth, grille_h)),
            origin=Origin(xyz=(x, grille_y, grille_cz)),
            material=cream,
            name=f"grille_side_rail_{side}",
        )
    slat_count = 15
    for i in range(slat_count):
        z = grille_cz - 0.052 + i * 0.0074
        _add_front_bar(
            cabinet,
            cream_high,
            f"grille_rib_{i}",
            0.0,
            z,
            grille_w,
            0.0038,
            y=FRONT_Y - 0.023,
            depth=0.010,
        )

    # Horizontal tuning dial below the grille with red/black AM/FM tick rows.
    cabinet.visual(
        Box((0.260, 0.004, 0.046)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.030, 0.074)),
        material=dial_face,
        name="tuning_dial_scale",
    )
    cabinet.visual(
        Box((0.248, 0.002, 0.034)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.033, 0.074)),
        material=model.material("smoky_dial_glass", rgba=(0.10, 0.105, 0.095, 1.0)),
        name="dial_glass_window",
    )
    tick_xs = (-0.100, -0.075, -0.050, -0.025, 0.0, 0.025, 0.050, 0.075, 0.100)
    for i, x in enumerate(tick_xs):
        mat = red_ink if i in (0, 4, 8) else dial_ink
        height = 0.020 if i in (0, 4, 8) else 0.012
        _add_front_bar(
            cabinet,
            mat,
            f"fm_tick_{i}",
            x,
            0.084,
            0.0025,
            height,
            y=FRONT_Y - 0.0345,
            depth=0.003,
        )
        _add_front_bar(
            cabinet,
            mat,
            f"am_tick_{i}",
            x,
            0.064,
            0.0025,
            height * 0.82,
            y=FRONT_Y - 0.0345,
            depth=0.003,
        )
    cabinet.visual(
        Box((0.004, 0.004, 0.040)),
        origin=Origin(xyz=(0.012, FRONT_Y - 0.0345, 0.074)),
        material=red_ink,
        name="red_tuning_pointer",
    )
    # Block-built AM/FM letter hints and small frequency-number blocks.
    _add_front_bar(cabinet, dial_ink, "fm_letter_f_stem", -0.120, 0.086, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "fm_letter_f_top", -0.116, 0.092, 0.008, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "fm_letter_f_mid", -0.117, 0.086, 0.006, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "fm_letter_m_left", -0.106, 0.086, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "fm_letter_m_right", -0.097, 0.086, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "fm_letter_m_bridge", -0.1015, 0.091, 0.007, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_a_left", -0.120, 0.064, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_a_right", -0.112, 0.064, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_a_cross", -0.116, 0.066, 0.008, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_m_left", -0.103, 0.064, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_m_right", -0.094, 0.064, 0.002, 0.014, y=FRONT_Y - 0.0345, depth=0.003)
    _add_front_bar(cabinet, dial_ink, "am_letter_m_bridge", -0.0985, 0.069, 0.007, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
    for i, x in enumerate((-0.065, -0.035, -0.005, 0.030, 0.060, 0.092)):
        _add_front_bar(cabinet, dial_ink, f"frequency_block_{i}", x, 0.092, 0.012, 0.002, y=FRONT_Y - 0.0345, depth=0.003)
        _add_front_bar(cabinet, dial_ink, f"am_number_block_{i}", x, 0.057, 0.014, 0.002, y=FRONT_Y - 0.0345, depth=0.003)

    # Two separate chrome rotary controls.  The flutes make their revolute
    # articulation visually meaningful even though the caps are round.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.026,
            body_style="skirted",
            top_diameter=0.044,
            edge_radius=0.0012,
            skirt=KnobSkirt(0.062, 0.006, flare=0.04, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=28, depth=0.0014),
            center=False,
        ),
        "fluted_chrome_knob",
    )

    def add_knob(name: str, x: float) -> None:
        knob = model.part(name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="chrome_fluted_body",
        )
        return knob

    tuning_knob = add_knob("tuning_knob", -0.195)
    volume_knob = add_knob("volume_knob", 0.195)

    model.articulation(
        "cabinet_to_tuning",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tuning_knob,
        origin=Origin(xyz=(-0.195, FRONT_Y - 0.026, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=3.5, lower=-3.14, upper=3.14),
    )
    model.articulation(
        "cabinet_to_volume",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=volume_knob,
        origin=Origin(xyz=(0.195, FRONT_Y - 0.026, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=0.0, upper=4.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    tuning = object_model.get_part("tuning_knob")
    volume = object_model.get_part("volume_knob")
    tuning_joint = object_model.get_articulation("cabinet_to_tuning")
    volume_joint = object_model.get_articulation("cabinet_to_volume")

    ctx.check(
        "cabinet is the single structural root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "cabinet",
        details=f"roots={[p.name for p in object_model.root_parts()]}",
    )
    ctx.check(
        "two chrome knobs are articulated controls",
        tuning_joint is not None and volume_joint is not None,
        details="missing tuning or volume revolute joint",
    )

    # Grille/dial layout: the ribbed speaker grille sits above the horizontal
    # tuning scale, and the knobs flank that scale on the lower control strip.
    grille_aabb = ctx.part_element_world_aabb(cabinet, elem="grille_rib_7")
    dial_aabb = ctx.part_element_world_aabb(cabinet, elem="tuning_dial_scale")
    left_aabb = ctx.part_world_aabb(tuning)
    right_aabb = ctx.part_world_aabb(volume)
    layout_ok = False
    if grille_aabb and dial_aabb and left_aabb and right_aabb:
        layout_ok = (
            grille_aabb[0][2] > dial_aabb[1][2]
            and left_aabb[1][0] < dial_aabb[0][0]
            and right_aabb[0][0] > dial_aabb[1][0]
        )
    ctx.check(
        "grille above dial with knobs flanking",
        layout_ok,
        details=f"grille={grille_aabb}, dial={dial_aabb}, tuning={left_aabb}, volume={right_aabb}",
    )

    ctx.expect_gap(
        cabinet,
        tuning,
        axis="y",
        positive_elem="lower_control_strip",
        max_gap=0.002,
        max_penetration=0.0002,
        name="tuning knob seats on control strip",
    )
    ctx.expect_gap(
        cabinet,
        volume,
        axis="y",
        positive_elem="lower_control_strip",
        max_gap=0.002,
        max_penetration=0.0002,
        name="volume knob seats on control strip",
    )

    rest_tuning = ctx.part_world_aabb(tuning)
    with ctx.pose({tuning_joint: 1.0, volume_joint: 1.2}):
        posed_tuning = ctx.part_world_aabb(tuning)
        ctx.check(
            "rotary controls remain mounted while turned",
            rest_tuning is not None and posed_tuning is not None and abs(posed_tuning[1][1] - rest_tuning[1][1]) < 0.003,
            details=f"rest={rest_tuning}, posed={posed_tuning}",
        )

    return ctx.report()


object_model = build_object_model()
