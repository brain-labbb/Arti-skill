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


FLANGE_OUTER_R = 0.180
FLANGE_INNER_R = 0.108
FLANGE_H = 0.024
CUP_OUTER_R = 0.088
CUP_INNER_R = 0.070
CUP_H = 0.125
CUP_BOTTOM_H = 0.014
TOOTH_COUNT = 60


def annular_cylinder(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )


def hollow_cup() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(CUP_OUTER_R).extrude(CUP_H)
    void = (
        cq.Workplane("XY")
        .workplane(offset=CUP_BOTTOM_H)
        .circle(CUP_INNER_R)
        .extrude(CUP_H - CUP_BOTTOM_H + 0.002)
    )
    lip = annular_cylinder(CUP_OUTER_R + 0.004, CUP_INNER_R - 0.002, 0.006).translate(
        (0, 0, CUP_H - 0.006)
    )
    seating_collar = annular_cylinder(0.108, CUP_OUTER_R - 0.002, 0.006).translate(
        (0, 0, 0.0)
    )
    return outer.cut(void).union(lip).union(seating_collar)


def elliptical_cam() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(0.055, 0.038)
        .extrude(0.018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harmonic_drive_reducer")

    gray_green = Material("gray_green_anodized", color=(0.38, 0.48, 0.42, 1.0))
    dark = Material("dark_bolt_steel", color=(0.03, 0.035, 0.04, 1.0))
    gold = Material("tan_gold_flexspline", color=(0.78, 0.58, 0.30, 1.0))
    tooth_gray = Material("fine_gray_teeth", color=(0.55, 0.56, 0.55, 1.0))
    charcoal = Material("charcoal_bearing", color=(0.08, 0.085, 0.09, 1.0))
    model.materials.extend([gray_green, dark, gold, tooth_gray, charcoal])

    flange = model.part("flange")
    flange.visual(
        mesh_from_cadquery(annular_cylinder(FLANGE_OUTER_R, FLANGE_INNER_R, FLANGE_H), "flange_ring"),
        material=gray_green,
        name="flange_ring",
    )
    # A shallow inner circular-spline ledge, visible as the fixed toothed housing bore.
    flange.visual(
        mesh_from_cadquery(annular_cylinder(0.118, 0.108, 0.040), "circular_spline_bore"),
        origin=Origin(xyz=(0, 0, FLANGE_H)),
        material=gray_green,
        name="circular_spline_bore",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12
        r = 0.148
        flange.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(r * math.cos(a), r * math.sin(a), FLANGE_H + 0.003)),
            material=dark,
            name=f"bolt_{i}",
        )

    flexspline = model.part("flexspline")
    flexspline.visual(
        mesh_from_cadquery(hollow_cup(), "flexspline_cup"),
        origin=Origin(xyz=(0, 0, FLANGE_H)),
        material=gold,
        name="flexspline_cup",
    )
    for i in range(TOOTH_COUNT):
        a = 2.0 * math.pi * i / TOOTH_COUNT
        r = CUP_OUTER_R + 0.0005
        flexspline.visual(
            Box((0.0035, 0.0070, 0.088)),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), FLANGE_H + 0.060),
                rpy=(0, 0, a),
            ),
            material=tooth_gray,
            name=f"tooth_{i}",
        )

    wave = model.part("wave_generator")
    wave.visual(
        mesh_from_cadquery(elliptical_cam(), "elliptical_cam"),
        origin=Origin(xyz=(0, 0, FLANGE_H + CUP_BOTTOM_H)),
        material=charcoal,
        name="elliptical_cam",
    )
    wave.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(xyz=(0, 0, FLANGE_H + CUP_BOTTOM_H + 0.0295)),
        material=dark,
        name="center_hub",
    )

    model.articulation(
        "step_01_flange_to_flexspline",
        ArticulationType.PRISMATIC,
        parent=flange,
        child=flexspline,
        origin=Origin(),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=40, velocity=0.15, lower=0.0, upper=0.100),
    )
    model.articulation(
        "step_02_flexspline_to_wave_generator",
        ArticulationType.PRISMATIC,
        parent=flexspline,
        child=wave,
        origin=Origin(),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=30, velocity=0.12, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    flange = object_model.get_part("flange")
    flex = object_model.get_part("flexspline")
    wave = object_model.get_part("wave_generator")
    flex_slide = object_model.get_articulation("step_01_flange_to_flexspline")
    wave_slide = object_model.get_articulation("step_02_flexspline_to_wave_generator")

    ctx.check(
        "three visible mechanical parts only",
        [part.name for part in object_model.parts] == ["flange", "flexspline", "wave_generator"],
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "ordered serial joint chain",
        [
            (joint.name, joint.parent, joint.child)
            for joint in object_model.articulations
        ]
        == [
            ("step_01_flange_to_flexspline", "flange", "flexspline"),
            ("step_02_flexspline_to_wave_generator", "flexspline", "wave_generator"),
        ],
        details=(
            "expected flange -> flexspline -> wave_generator; "
            f"got {[(joint.name, joint.parent, joint.child) for joint in object_model.articulations]}"
        ),
    )
    ctx.check(
        "all movable joints translate in the same +Z direction",
        all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in object_model.articulations),
        details=f"axes={[(joint.name, joint.axis) for joint in object_model.articulations]}",
    )

    ctx.allow_overlap(
        flex,
        wave,
        elem_a="flexspline_cup",
        elem_b="elliptical_cam",
        reason="The wave-generator cam is intentionally seated down inside the hollow cup bore in the assembled reducer.",
    )
    ctx.allow_overlap(
        flange,
        flex,
        elem_a="circular_spline_bore",
        elem_b="flexspline_cup",
        reason="The cup's toothed outside is intentionally nested in the fixed circular-spline bore at the assembled pose.",
    )

    ctx.expect_origin_distance(flex, flange, axes="xy", max_dist=0.0005, name="flexspline concentric with flange")
    ctx.expect_origin_distance(wave, flange, axes="xy", max_dist=0.0005, name="wave generator concentric with flange")
    ctx.expect_within(
        flex,
        flange,
        axes="xy",
        inner_elem="flexspline_cup",
        outer_elem="flange_ring",
        margin=0.0,
        name="cup nests inside flange opening footprint",
    )
    ctx.expect_within(
        wave,
        flex,
        axes="xy",
        inner_elem="elliptical_cam",
        outer_elem="flexspline_cup",
        margin=0.0,
        name="elliptical cam fits inside cup bore",
    )
    ctx.expect_overlap(flex, flange, axes="z", elem_a="flexspline_cup", elem_b="circular_spline_bore", min_overlap=0.035, name="assembled cup sits through fixed spline bore")
    ctx.expect_joint_motion_axis(
        flex_slide,
        flex,
        world_axis="z",
        direction="positive",
        min_delta=0.090,
        q0=0.0,
        q1=0.100,
        name="step 01 flexspline moves upward",
    )
    ctx.expect_joint_motion_axis(
        wave_slide,
        wave,
        world_axis="z",
        direction="positive",
        min_delta=0.160,
        q0=0.0,
        q1=0.180,
        name="step 02 wave generator moves upward relative to flexspline",
    )

    with ctx.pose({flex_slide: 0.055, wave_slide: 0.175}):
        ctx.expect_gap(flex, flange, axis="z", positive_elem="flexspline_cup", negative_elem="flange_ring", min_gap=0.045, name="step 01 flexspline lifts upward from fixed flange")
        ctx.expect_gap(wave, flex, axis="z", positive_elem="elliptical_cam", negative_elem="flexspline_cup", min_gap=0.025, name="step 02 wave generator continues upward after flexspline")
        ctx.expect_origin_distance(flex, flange, axes="xy", max_dist=0.0005, name="exploded flexspline remains vertical")
        ctx.expect_origin_distance(wave, flange, axes="xy", max_dist=0.0005, name="exploded wave remains vertical")

    return ctx.report()


object_model = build_object_model()
