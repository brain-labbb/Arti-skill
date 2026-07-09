from __future__ import annotations

import math

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, MotionLimits, Origin, TestContext, TestReport


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> str:
    model.material(name, rgba=rgba)
    return name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_zipper_slider")
    metal = _mat(model, "brushed_silver", (0.62, 0.61, 0.57, 1.0))
    dark = _mat(model, "dark_shadow", (0.03, 0.03, 0.032, 1.0))
    tape = _mat(model, "black_fabric_tape", (0.012, 0.012, 0.014, 1.0))

    track = model.part("zipper_track")
    track.visual(Box((0.020, 0.220, 0.004)), origin=Origin(xyz=(-0.018, 0.0, 0.002)), material=tape, name="fabric_tape_0")
    track.visual(Box((0.020, 0.220, 0.004)), origin=Origin(xyz=(0.018, 0.0, 0.002)), material=tape, name="fabric_tape_1")
    for i in range(14):
        y = -0.091 + i * 0.014
        track.visual(Box((0.010, 0.006, 0.006)), origin=Origin(xyz=(-0.005, y, 0.007), rpy=(0.0, 0.0, 0.15)), material=metal, name=f"tooth_left_{i}")
        track.visual(Box((0.010, 0.006, 0.006)), origin=Origin(xyz=(0.005, y + 0.007, 0.007), rpy=(0.0, 0.0, -0.15)), material=metal, name=f"tooth_right_{i}")
    track.visual(Box((0.055, 0.012, 0.007)), origin=Origin(xyz=(0.0, -0.110, 0.007)), material=metal, name="bottom_stop")

    slider = model.part("slider_body")
    slider.visual(Box((0.060, 0.048, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=metal, name="slider_shell")
    slider.visual(Box((0.028, 0.060, 0.010)), origin=Origin(xyz=(0.0, 0.005, 0.027)), material=metal, name="central_bridge")
    slider.visual(Box((0.014, 0.056, 0.006)), origin=Origin(xyz=(0.0, 0.000, 0.006)), material=dark, name="tooth_channel")
    slider.visual(Cylinder(radius=0.010, length=0.045), origin=Origin(xyz=(0.0, 0.020, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)), material=metal, name="pull_pivot_bar")
    slider.visual(Box((0.040, 0.008, 0.008)), origin=Origin(xyz=(0.0, -0.026, 0.015)), material=metal, name="rear_crimp")

    pull = model.part("pull_tab")
    pull.visual(Box((0.052, 0.105, 0.010)), origin=Origin(xyz=(0.0, -0.058, 0.000)), material=metal, name="long_pull_plate")
    pull.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(0.0, -0.108, 0.000)), material=metal, name="rounded_pull_end")
    pull.visual(Cylinder(radius=0.014, length=0.012), origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark, name="pivot_hole")
    pull.visual(Box((0.026, 0.016, 0.012)), origin=Origin(xyz=(0.0, -0.004, 0.000)), material=metal, name="neck_block")

    model.articulation(
        "track_to_slider",
        ArticulationType.PRISMATIC,
        parent=track,
        child=slider,
        origin=Origin(xyz=(0.0, -0.045, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=0.090),
    )
    model.articulation(
        "slider_to_pull_tab",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=pull,
        origin=Origin(xyz=(0.0, 0.020, 0.040)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.9, upper=1.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    track = object_model.get_part("zipper_track")
    slider = object_model.get_part("slider_body")
    pull = object_model.get_part("pull_tab")
    slide = object_model.get_articulation("track_to_slider")
    hinge = object_model.get_articulation("slider_to_pull_tab")

    ctx.allow_overlap(slider, track, elem_a="tooth_channel", elem_b="tooth_left_6", reason="The channel intentionally surrounds the zipper teeth where the slider rides.")
    ctx.allow_overlap(slider, track, elem_a="tooth_channel", elem_b="tooth_right_6", reason="The channel intentionally surrounds the zipper teeth where the slider rides.")
    ctx.allow_overlap(slider, pull, elem_a="pull_pivot_bar", elem_b="pivot_hole", reason="The pull tab pivot hole intentionally wraps around the slider pivot bar.")
    ctx.allow_overlap(pull, slider, elem_a="neck_block", elem_b="pull_pivot_bar", reason="The pull tab neck is locally wrapped around the pivot bar in this simplified metal loop.")
    ctx.allow_overlap(pull, slider, elem_a="pivot_hole", elem_b="central_bridge", reason="The pull tab pivot loop locally nests around the raised bridge of the metal slider.")
    ctx.expect_within(slider, track, axes="x", inner_elem="tooth_channel", outer_elem="bottom_stop", margin=0.025, name="slider channel centered on tooth line")
    ctx.expect_contact(pull, slider, elem_a="pivot_hole", elem_b="pull_pivot_bar", contact_tol=0.005, name="pull tab pivots on bar")

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.080}):
        moved_pos = ctx.part_world_position(slider)
    ctx.check("slider moves along zipper track", rest_pos is not None and moved_pos is not None and moved_pos[1] > rest_pos[1] + 0.06, details=f"rest={rest_pos}, moved={moved_pos}")

    rest_pull = ctx.part_element_world_aabb(pull, elem="long_pull_plate")
    with ctx.pose({hinge: 0.8}):
        lifted_pull = ctx.part_element_world_aabb(pull, elem="long_pull_plate")
    rest_top = rest_pull[1][2] if rest_pull else None
    lifted_top = lifted_pull[1][2] if lifted_pull else None
    ctx.check("pull tab rotates upward", rest_top is not None and lifted_top is not None and lifted_top > rest_top + 0.02, details=f"rest_top={rest_top}, lifted_top={lifted_top}")
    ctx.check("zipper teeth are modeled", len(track.visuals) >= 28, details="track should include paired interlocking teeth")
    return ctx.report()


object_model = build_object_model()
