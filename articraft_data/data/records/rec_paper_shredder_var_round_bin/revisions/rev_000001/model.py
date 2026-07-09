from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translate_profile(profile, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular plastic housing."""
    safe_radius = min(radius, min(size) * 0.35)
    return cq.Workplane("XY").box(*size).edges().fillet(safe_radius)


def _cylinder_bin_shell(
    outer_r: float,
    height: float,
    wall: float,
    cutout_half_w: float,
    cutout_z_lo: float,
    cutout_z_hi: float,
) -> cq.Workplane:
    """Hollow cylindrical bin wall with rectangular front basket opening."""
    shell = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(outer_r - wall)
        .extrude(height)
    )
    cut_h = cutout_z_hi - cutout_z_lo
    cut_z = (cutout_z_lo + cutout_z_hi) / 2.0
    cut_depth = wall * 5.0
    cutter = (
        cq.Workplane("XY")
        .box(cutout_half_w * 2.0, cut_depth, cut_h)
        .translate((0.0, outer_r, cut_z))
    )
    return shell.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_paper_shredder")

    charcoal = model.material("charcoal_plastic", rgba=(0.015, 0.016, 0.016, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.035, 0.037, 0.035, 1.0))
    gloss_black = model.material("gloss_black_plastic", rgba=(0.0, 0.0, 0.0, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    smoked = model.material("smoked_window_plastic", rgba=(0.06, 0.075, 0.08, 0.46))
    pale_icon = model.material("pale_indicator_marks", rgba=(0.82, 0.84, 0.80, 1.0))

    shell = model.part("shell")

    # Cylindrical drum bin shell with rectangular front basket opening for the basket.
    bin_r = 0.195
    bin_height = 0.400
    bin_wall = 0.018
    basket_opening_half_w = 0.155
    basket_opening_z_lo = 0.068
    basket_opening_z_hi = 0.395
    bin_shell = mesh_from_cadquery(
        _cylinder_bin_shell(
            bin_r, bin_height, bin_wall,
            basket_opening_half_w, basket_opening_z_lo, basket_opening_z_hi,
        ),
        "cylinder_shell",
    )
    shell.visual(
        bin_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=charcoal,
        name="cylinder_wall",
    )
    # Circular base plate (solid disk under the cylinder wall).
    shell.visual(
        Cylinder(radius=bin_r, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=satin_black,
        name="base_plate",
    )

    # Heavy cylindrical shredder head/lid sitting on the cylindrical bin.
    head_r = bin_r + 0.008  # slightly larger than bin for overhang
    head_h = 0.110
    shell.visual(
        Cylinder(radius=head_r, length=head_h),
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
        material=satin_black,
        name="top_head",
    )
    shell.visual(
        Cylinder(radius=0.047, length=0.350),
        origin=Origin(xyz=(0.0, head_r + 0.005, 0.475), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="rounded_front_apron",
    )
    shell.visual(
        Box((0.338, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.120, 0.560)),
        material=gloss_black,
        name="shredder_throat",
    )
    shell.visual(
        Cylinder(radius=0.010, length=0.335),
        origin=Origin(xyz=(0.0, 0.105, 0.548), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="cutter_roller_0",
    )
    shell.visual(
        Cylinder(radius=0.010, length=0.335),
        origin=Origin(xyz=(0.0, 0.126, 0.544), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="cutter_roller_1",
    )

    # Top feed slot, control panel, and blank oval front badge (no text labels).
    shell.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.290, 0.185, 0.020), 0.004),
            "top_recess_panel",
        ),
        origin=Origin(xyz=(0.0, 0.018, 0.562)),
        material=charcoal,
        name="top_recess_panel",
    )
    shell.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.255, 0.018, 0.008), 0.005),
            "feed_slot",
        ),
        origin=Origin(xyz=(0.0, 0.030, 0.566)),
        material=gloss_black,
        name="feed_slot",
    )
    shell.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.165, 0.036, 0.010), 0.004),
            "control_recess",
        ),
        origin=Origin(xyz=(0.035, -0.088, 0.566)),
        material=gloss_black,
        name="control_recess",
    )
    shell.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.130, 0.054, exponent=2.0), 0.004),
            "front_badge",
        ),
        origin=Origin(xyz=(0.0, bin_r + 0.002, 0.510), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_badge",
    )
    for i, x in enumerate([-0.088, -0.064, -0.040, -0.016, 0.008, 0.032, 0.056, 0.080]):
        shell.visual(
            Cylinder(radius=0.0032, length=0.0040),
            origin=Origin(xyz=(x, 0.000, 0.562)),
            material=pale_icon,
            name=f"indicator_dot_{i}",
        )

    # Four small caster yokes are fixed to the shell; the wheels are rotating child parts.
    # Positions must fit within the circular base plate (radius 0.195).
    caster_positions = [
        (-0.120, 0.120, 0.019),
        (0.120, 0.120, 0.019),
        (-0.120, -0.120, 0.019),
        (0.120, -0.120, 0.019),
    ]
    for i, (x, y, z) in enumerate(caster_positions):
        shell.visual(
            Box((0.018, 0.018, 0.030)),
            origin=Origin(xyz=(x, y, 0.070)),
            material=gloss_black,
            name=f"caster_stem_{i}",
        )
        shell.visual(
            Box((0.052, 0.024, 0.010)),
            origin=Origin(xyz=(x, y, 0.060)),
            material=gloss_black,
            name=f"caster_bridge_{i}",
        )
        shell.visual(
            Box((0.008, 0.021, 0.050)),
            origin=Origin(xyz=(x - 0.018, y, 0.031)),
            material=gloss_black,
            name=f"caster_fork_{i}_0",
        )
        shell.visual(
            Box((0.008, 0.021, 0.050)),
            origin=Origin(xyz=(x + 0.018, y, 0.031)),
            material=gloss_black,
            name=f"caster_fork_{i}_1",
        )

    # Removable waste basket / drawer.  At q=0 it is partially pulled out like the reference.
    basket = model.part("basket")
    front_outer = rounded_rect_profile(0.338, 0.350, 0.022)
    window_hole = _translate_profile(rounded_rect_profile(0.064, 0.218, 0.031), dx=-0.060, dy=-0.010)
    basket.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(front_outer, [window_hole], 0.018),
            "basket_front_panel",
        ),
        origin=Origin(xyz=(0.0, 0.140, 0.020), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_panel",
    )
    basket.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.056, 0.202, 0.028), 0.006),
            "view_window",
        ),
        origin=Origin(xyz=(-0.060, 0.128, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=smoked,
        name="view_window",
    )
    basket.visual(
        Box((0.020, 0.014, 0.320)),
        origin=Origin(xyz=(0.146, 0.151, 0.020)),
        material=gloss_black,
        name="front_grip",
    )
    basket.visual(
        Box((0.014, 0.012, 0.320)),
        origin=Origin(xyz=(-0.158, 0.149, 0.020)),
        material=gloss_black,
        name="edge_lip",
    )
    basket.visual(
        Box((0.300, 0.205, 0.022)),
        origin=Origin(xyz=(0.0, -0.060, -0.130)),
        material=gloss_black,
        name="bottom_tray",
    )
    basket.visual(
        Box((0.018, 0.205, 0.260)),
        origin=Origin(xyz=(0.155, -0.060, -0.005)),
        material=gloss_black,
        name="side_wall_0",
    )
    basket.visual(
        Box((0.018, 0.205, 0.260)),
        origin=Origin(xyz=(-0.155, -0.060, -0.005)),
        material=gloss_black,
        name="side_wall_1",
    )
    basket.visual(
        Box((0.300, 0.018, 0.260)),
        origin=Origin(xyz=(0.0, -0.170, -0.005)),
        material=gloss_black,
        name="back_wall",
    )
    for suffix, x in (("0", 0.155), ("1", -0.155)):
        basket.visual(
            Box((0.018, 0.162, 0.250)),
            origin=Origin(xyz=(x, 0.050, -0.005)),
            material=gloss_black,
            name=f"front_side_post_{suffix}",
        )
    for suffix, x in (("0", 0.164), ("1", -0.164)):
        basket.visual(
            Box((0.012, 0.205, 0.020)),
            origin=Origin(xyz=(x, -0.060, -0.100)),
            material=gloss_black,
            name=f"slide_runner_{suffix}",
        )

    model.articulation(
        "shell_to_basket",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=basket,
        origin=Origin(xyz=(0.0, 0.080, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.28, lower=-0.045, upper=0.130),
    )

    # Moving top controls.
    button_specs = [("button_0", -0.010, -0.088), ("button_1", 0.016, -0.088)]
    for name, x, y in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=pale_icon,
            name="button_cap",
        )
        model.articulation(
            f"shell_to_{name}",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=button,
            origin=Origin(xyz=(x, y, 0.563)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.004),
        )

    switch = model.part("control_switch")
    switch.visual(
        mesh_from_cadquery(_rounded_box((0.032, 0.014, 0.006), 0.003), "switch_slider"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=pale_icon,
        name="slider_tab",
    )
    model.articulation(
        "shell_to_switch",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=switch,
        origin=Origin(xyz=(0.053, -0.088, 0.563)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=-0.022, upper=0.022),
    )

    for i, (x, y, z) in enumerate(caster_positions):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.018, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="wheel",
        )
        model.articulation(
            f"shell_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=caster,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    basket = object_model.get_part("basket")
    basket_slide = object_model.get_articulation("shell_to_basket")
    switch_slide = object_model.get_articulation("shell_to_switch")
    button_0 = object_model.get_articulation("shell_to_button_0")
    caster_0 = object_model.get_part("caster_0")

    # The basket slides inside the cylindrical shell; the bottom_tray intentionally
    # overlaps with the cylinder_wall as it sits within the round bin envelope.
    ctx.allow_overlap(
        basket,
        shell,
        elem_a="bottom_tray",
        elem_b="cylinder_wall",
        reason="Basket bottom_tray slides inside the cylindrical bin shell; overlap is the retained insertion.",
    )
    ctx.allow_overlap(
        basket,
        shell,
        elem_a="side_wall_0",
        elem_b="cylinder_wall",
        reason="Basket side_wall_0 slides inside the cylindrical bin shell; overlap is the retained insertion.",
    )
    ctx.allow_overlap(
        basket,
        shell,
        elem_a="side_wall_1",
        elem_b="cylinder_wall",
        reason="Basket side_wall_1 slides inside the cylindrical bin shell; overlap is the retained insertion.",
    )
    ctx.allow_overlap(
        basket,
        shell,
        elem_a="back_wall",
        elem_b="cylinder_wall",
        reason="Basket back_wall slides inside the cylindrical bin shell; overlap is the retained insertion.",
    )
    for elem_name in ("slide_runner_0", "slide_runner_1", "front_side_post_0", "front_side_post_1"):
        ctx.allow_overlap(
            basket,
            shell,
            elem_a=elem_name,
            elem_b="cylinder_wall",
            reason=f"Basket {elem_name} slides inside the cylindrical bin shell; overlap is the retained insertion.",
        )

    ctx.expect_gap(
        basket,
        shell,
        axis="y",
        positive_elem="front_panel",
        negative_elem="top_head",
        min_gap=0.006,
        name="basket front is visibly pulled past the head",
    )
    ctx.expect_overlap(
        basket,
        shell,
        axes="x",
        elem_a="bottom_tray",
        elem_b="base_plate",
        min_overlap=0.260,
        name="basket tray fits within the cylindrical base",
    )
    ctx.expect_gap(
        basket,
        shell,
        axis="z",
        positive_elem="bottom_tray",
        negative_elem="base_plate",
        min_gap=0.004,
        name="basket bottom rides above the base plate",
    )
    ctx.expect_within(
        basket,
        basket,
        axes="x",
        inner_elem="view_window",
        outer_elem="front_panel",
        margin=0.002,
        name="viewing window is contained in the basket front",
    )
    # Prove the cylindrical bin envelope contains the basket within the round shell.
    ctx.expect_within(
        basket,
        shell,
        axes="x",
        inner_elem="bottom_tray",
        outer_elem="cylinder_wall",
        margin=0.005,
        name="basket fits within the cylindrical bin wall on X",
    )
    ctx.expect_within(
        basket,
        shell,
        axes="y",
        inner_elem="back_wall",
        outer_elem="cylinder_wall",
        margin=0.005,
        name="basket fits within the cylindrical bin wall on Y",
    )
    ctx.expect_overlap(
        shell,
        shell,
        axes="x",
        elem_a="feed_slot",
        elem_b="top_recess_panel",
        min_overlap=0.240,
        name="feed slot spans the top panel",
    )
    ctx.expect_contact(
        caster_0,
        shell,
        elem_a="wheel",
        elem_b="caster_fork_0_0",
        contact_tol=0.004,
        name="caster wheel is captured by its fork",
    )

    rest_basket = ctx.part_world_position(basket)
    with ctx.pose({basket_slide: 0.130}):
        extended_basket = ctx.part_world_position(basket)
        ctx.expect_overlap(
            basket,
            shell,
            axes="y",
            elem_a="bottom_tray",
            elem_b="base_plate",
            min_overlap=0.080,
            name="extended basket remains retained in the cylinder",
        )
    ctx.check(
        "basket slides outward on positive travel",
        rest_basket is not None
        and extended_basket is not None
        and extended_basket[1] > rest_basket[1] + 0.10,
        details=f"rest={rest_basket}, extended={extended_basket}",
    )

    rest_switch = ctx.part_world_position(object_model.get_part("control_switch"))
    with ctx.pose({switch_slide: 0.022}):
        moved_switch = ctx.part_world_position(object_model.get_part("control_switch"))
    ctx.check(
        "control switch slides across the panel",
        rest_switch is not None and moved_switch is not None and moved_switch[0] > rest_switch[0] + 0.015,
        details=f"rest={rest_switch}, moved={moved_switch}",
    )

    rest_button = ctx.part_world_position(object_model.get_part("button_0"))
    with ctx.pose({button_0: 0.004}):
        pressed_button = ctx.part_world_position(object_model.get_part("button_0"))
    ctx.check(
        "push button depresses downward",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
