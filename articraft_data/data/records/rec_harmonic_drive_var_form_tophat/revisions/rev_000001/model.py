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


# ── Flange / circular-spline housing ──
FLANGE_OUTER_R = 0.180
FLANGE_INNER_R = 0.108
FLANGE_H = 0.024

# ── Top-hat flexspline barrel + brim ──
BARREL_OUTER_R = 0.088
BARREL_INNER_R = 0.076
# Barrel height chosen so brim bottom sits on the circular-spline bore top face:
# brim_bottom = BARREL_H - BRIM_H  ==  bore_top = 0.040  →  BARREL_H = 0.047
BARREL_H = 0.047
BRIM_OUTER_R = 0.135
BRIM_H = 0.007
BRIM_BOLT_COUNT = 8
SEAT_INNER_R = 0.020
SEAT_H = 0.004

# ── Tooth band ──
TOOTH_COUNT = 60
TOOTH_H = 0.032

# ── Wave generator ──
# WG_Z_BASE set so cam bottom contacts the seat top face:
# cam_world_min_z = FLANGE_H + WG_Z_BASE  ==  FLANGE_H + SEAT_H  →  WG_Z_BASE = SEAT_H
WG_Z_BASE = SEAT_H  # 0.004
CAM_H = 0.018
HUB_LENGTH = 0.030


def annular_cylinder(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )


def top_hat_flexspline() -> cq.Workplane:
    """Open-ended thin-wall barrel with outward radial mounting brim at the top.

    The barrel is a hollow annular cylinder open at both ends.  The brim is a
    wider annular ring sitting at the top of the barrel, forming the classic
    top-hat / silk-hat silhouette used by SHG/CSG-series harmonic drives.
    A thin internal seat ring at the barrel bottom registers the wave
    generator and provides a physical support path.
    """
    barrel = annular_cylinder(BARREL_OUTER_R, BARREL_INNER_R, BARREL_H)
    brim = annular_cylinder(BRIM_OUTER_R, BARREL_INNER_R, BRIM_H).translate(
        (0, 0, BARREL_H - BRIM_H)
    )
    # Internal seat ring at barrel bottom: the wave-generator cam rests on this.
    seat = annular_cylinder(BARREL_INNER_R, SEAT_INNER_R, SEAT_H)
    return barrel.union(brim).union(seat)


def elliptical_cam() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(0.055, 0.038)
        .extrude(CAM_H)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harmonic_drive_reducer")

    gray_green = Material("gray_green_anodized", color=(0.38, 0.48, 0.42, 1.0))
    dark = Material("dark_bolt_steel", color=(0.03, 0.035, 0.04, 1.0))
    gold = Material("tan_gold_flexspline", color=(0.78, 0.58, 0.30, 1.0))
    tooth_gray = Material("fine_gray_teeth", color=(0.55, 0.56, 0.55, 1.0))
    charcoal = Material("charcoal_bearing", color=(0.08, 0.085, 0.09, 1.0))
    model.materials.extend([gray_green, dark, gold, tooth_gray, charcoal])

    # ── Flange (fixed base / circular-spline housing) ──
    flange = model.part("flange")
    flange.visual(
        mesh_from_cadquery(
            annular_cylinder(FLANGE_OUTER_R, FLANGE_INNER_R, FLANGE_H),
            "flange_ring",
        ),
        material=gray_green,
        name="flange_ring",
    )
    # Shallow circular-spline bore ledge visible above the flange face.
    flange.visual(
        mesh_from_cadquery(
            annular_cylinder(0.118, 0.108, 0.040),
            "circular_spline_bore",
        ),
        origin=Origin(xyz=(0, 0, FLANGE_H)),
        material=gray_green,
        name="circular_spline_bore",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12
        r = 0.148
        flange.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), FLANGE_H + 0.003)
            ),
            material=dark,
            name=f"bolt_{i}",
        )

    # ── Flexspline (top-hat / silk-hat) ──
    flexspline = model.part("flexspline")
    flexspline.visual(
        mesh_from_cadquery(top_hat_flexspline(), "flexspline_cup"),
        origin=Origin(xyz=(0, 0, FLANGE_H)),
        material=gold,
        name="flexspline_cup",
    )
    # Bolt heads on the brim top face (mounting bolt circle).
    for i in range(BRIM_BOLT_COUNT):
        a = 2.0 * math.pi * i / BRIM_BOLT_COUNT
        r = 0.110
        flexspline.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(
                xyz=(
                    r * math.cos(a),
                    r * math.sin(a),
                    FLANGE_H + BARREL_H + 0.0025,
                )
            ),
            material=dark,
            name=f"brim_bolt_{i}",
        )
    # External tooth band on the barrel, centred below the brim.
    tooth_z_center = FLANGE_H + (BARREL_H - BRIM_H) / 2.0
    for i in range(TOOTH_COUNT):
        a = 2.0 * math.pi * i / TOOTH_COUNT
        r = BARREL_OUTER_R + 0.0005
        flexspline.visual(
            Box((0.0035, 0.0070, TOOTH_H)),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), tooth_z_center),
                rpy=(0, 0, a),
            ),
            material=tooth_gray,
            name=f"tooth_{i}",
        )

    # ── Wave generator ──
    wave = model.part("wave_generator")
    wave.visual(
        mesh_from_cadquery(elliptical_cam(), "elliptical_cam"),
        origin=Origin(xyz=(0, 0, FLANGE_H + WG_Z_BASE)),
        material=charcoal,
        name="elliptical_cam",
    )
    wave.visual(
        Cylinder(radius=0.018, length=HUB_LENGTH),
        origin=Origin(
            xyz=(0, 0, FLANGE_H + WG_Z_BASE + CAM_H + HUB_LENGTH / 2)
        ),
        material=dark,
        name="center_hub",
    )

    # ── Articulations (coaxial +Z prismatic chain) ──
    model.articulation(
        "step_01_flange_to_flexspline",
        ArticulationType.PRISMATIC,
        parent=flange,
        child=flexspline,
        origin=Origin(),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(
            effort=40, velocity=0.15, lower=0.0, upper=0.100
        ),
    )
    model.articulation(
        "step_02_flexspline_to_wave_generator",
        ArticulationType.PRISMATIC,
        parent=flexspline,
        child=wave,
        origin=Origin(),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(
            effort=30, velocity=0.12, lower=0.0, upper=0.180
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    flange = object_model.get_part("flange")
    flex = object_model.get_part("flexspline")
    wave = object_model.get_part("wave_generator")
    flex_slide = object_model.get_articulation("step_01_flange_to_flexspline")
    wave_slide = object_model.get_articulation(
        "step_02_flexspline_to_wave_generator"
    )

    # ── Structural identity ──
    ctx.check(
        "three visible mechanical parts only",
        [p.name for p in object_model.parts]
        == ["flange", "flexspline", "wave_generator"],
        details=f"parts={[p.name for p in object_model.parts]}",
    )
    ctx.check(
        "ordered serial joint chain",
        [
            (j.name, j.parent, j.child)
            for j in object_model.articulations
        ]
        == [
            ("step_01_flange_to_flexspline", "flange", "flexspline"),
            ("step_02_flexspline_to_wave_generator", "flexspline", "wave_generator"),
        ],
        details=(
            "expected flange -> flexspline -> wave_generator; "
            f"got {[(j.name, j.parent, j.child) for j in object_model.articulations]}"
        ),
    )
    ctx.check(
        "all movable joints translate in the same +Z direction",
        all(
            tuple(j.axis) == (0.0, 0.0, 1.0)
            for j in object_model.articulations
        ),
        details=f"axes={[(j.name, j.axis) for j in object_model.articulations]}",
    )

    # ── Top-hat brim proof ──
    # The flexspline_cup mesh now includes the outward brim at BRIM_OUTER_R.
    # Without the brim the barrel alone projects to 2*BARREL_OUTER_R = 0.176 on
    # each axis; with the brim it projects to 2*BRIM_OUTER_R = 0.270.
    # The flange_ring projects to 2*FLANGE_OUTER_R = 0.360.
    # XY overlap of flexspline_cup ∩ flange_ring:
    #   with brim  → 0.270 (limited by brim)
    #   without    → 0.176 (limited by barrel)
    # So min_overlap = 0.25 proves the brim is present.
    ctx.expect_overlap(
        flex,
        flange,
        axes="xy",
        elem_a="flexspline_cup",
        elem_b="flange_ring",
        min_overlap=0.25,
        name="top-hat brim extends radially outward beyond barrel",
    )

    # ── Intentional overlaps ──
    ctx.allow_overlap(
        flex,
        wave,
        elem_a="flexspline_cup",
        elem_b="elliptical_cam",
        reason=(
            "The wave-generator cam sits on the internal seat ring inside the "
            "open top-hat barrel bore; the seat face and cam bottom face share "
            "a contact plane at the assembled pose."
        ),
    )
    ctx.allow_overlap(
        flange,
        flex,
        elem_a="circular_spline_bore",
        elem_b="flexspline_cup",
        reason=(
            "The top-hat brim seats directly on the circular-spline bore top "
            "face and the barrel nests inside the bore at the assembled pose."
        ),
    )

    # ── Concentric alignment ──
    ctx.expect_origin_distance(
        flex, flange, axes="xy", max_dist=0.0005,
        name="flexspline concentric with flange",
    )
    ctx.expect_origin_distance(
        wave, flange, axes="xy", max_dist=0.0005,
        name="wave generator concentric with flange",
    )

    # ── Assembled nesting ──
    ctx.expect_within(
        flex, flange, axes="xy",
        inner_elem="flexspline_cup",
        outer_elem="flange_ring",
        margin=0.0,
        name="top-hat barrel and brim fit within flange outer footprint",
    )
    ctx.expect_within(
        wave, flex, axes="xy",
        inner_elem="elliptical_cam",
        outer_elem="flexspline_cup",
        margin=0.0,
        name="elliptical cam fits inside top-hat barrel bore",
    )
    ctx.expect_overlap(
        flex, flange, axes="z",
        elem_a="flexspline_cup",
        elem_b="circular_spline_bore",
        min_overlap=0.035,
        name="assembled barrel sits through fixed spline bore",
    )

    # ── Joint motion direction (exact pose-based) ──
    rest_flex_z = ctx.part_world_position(flex)
    rest_wave_z = ctx.part_world_position(wave)

    with ctx.pose({flex_slide: 0.100}):
        ext_flex_z = ctx.part_world_position(flex)
    ctx.check(
        "step 01 flexspline moves upward (+Z)",
        rest_flex_z is not None
        and ext_flex_z is not None
        and ext_flex_z[2] > rest_flex_z[2] + 0.090,
        details=f"rest={rest_flex_z}, extended={ext_flex_z}",
    )

    with ctx.pose({wave_slide: 0.180}):
        ext_wave_z = ctx.part_world_position(wave)
    ctx.check(
        "step 02 wave generator moves upward (+Z) relative to flexspline",
        rest_wave_z is not None
        and ext_wave_z is not None
        and ext_wave_z[2] > rest_wave_z[2] + 0.160,
        details=f"rest={rest_wave_z}, extended={ext_wave_z}",
    )

    # ── Exploded pose: ordered Z gaps ──
    with ctx.pose({flex_slide: 0.055, wave_slide: 0.175}):
        ctx.expect_gap(
            flex, flange, axis="z",
            positive_elem="flexspline_cup",
            negative_elem="flange_ring",
            min_gap=0.045,
            name="step 01 flexspline lifts upward from fixed flange",
        )
        ctx.expect_gap(
            wave, flex, axis="z",
            positive_elem="elliptical_cam",
            negative_elem="flexspline_cup",
            min_gap=0.025,
            name="step 02 wave generator continues upward after flexspline",
        )
        ctx.expect_origin_distance(
            flex, flange, axes="xy", max_dist=0.0005,
            name="exploded flexspline remains vertical",
        )
        ctx.expect_origin_distance(
            wave, flange, axes="xy", max_dist=0.0005,
            name="exploded wave remains vertical",
        )

    return ctx.report()


object_model = build_object_model()
