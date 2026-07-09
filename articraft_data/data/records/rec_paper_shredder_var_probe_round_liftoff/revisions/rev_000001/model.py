from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------

def _bin_wall_and_bottom() -> cq.Workplane:
    """Hollow cylindrical bin wall with integral bottom plate."""
    outer_r = 0.135
    wall = 0.004
    height = 0.300
    bottom_t = 0.005

    outer = cq.Workplane("XY").circle(outer_r).extrude(height)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=bottom_t)
        .circle(outer_r - wall)
        .extrude(height - bottom_t)
    )
    return outer.cut(inner)


def _rim_ring() -> cq.Workplane:
    """Flat rim ring sitting on top of the bin wall."""
    outer_r = 0.135
    rim_w = 0.008
    rim_h = 0.012
    height = 0.300

    return (
        cq.Workplane("XY")
        .workplane(offset=height - rim_h)
        .circle(outer_r + rim_w)
        .circle(outer_r)
        .extrude(rim_h)
    )


def _head_housing() -> cq.Workplane:
    """Rounded shredder head disk (cylinder with domed top edge)."""
    disk_r = 0.143  # matches rim outer radius
    head_h = 0.088

    body = cq.Workplane("XY").circle(disk_r).extrude(head_h)
    try:
        body = body.edges(">Z").fillet(0.014)
    except Exception:
        pass
    return body


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_paper_shredder")

    # -- materials --
    charcoal = model.material("charcoal_plastic", rgba=(0.015, 0.016, 0.016, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.035, 0.037, 0.035, 1.0))
    gloss_black = model.material("gloss_black_plastic", rgba=(0.0, 0.0, 0.0, 1.0))
    pale_icon = model.material("pale_indicator_marks", rgba=(0.82, 0.84, 0.80, 1.0))
    rim_finish = model.material("dark_rim_finish", rgba=(0.08, 0.08, 0.09, 1.0))

    # ------------------------------------------------------------------
    # SHELL — round cylindrical bin body (root part)
    # ------------------------------------------------------------------
    shell = model.part("shell")

    shell.visual(
        mesh_from_cadquery(_bin_wall_and_bottom(), "bin_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="bin_wall",
    )
    shell.visual(
        mesh_from_cadquery(_rim_ring(), "rim_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rim_finish,
        name="rim_ring",
    )

    # ------------------------------------------------------------------
    # HEAD — shredder head that clamps onto the rim and lifts off
    # ------------------------------------------------------------------
    head = model.part("head")

    # Main housing disk (origin at rear hinge; disk centered at y=+0.143)
    head.visual(
        mesh_from_cadquery(_head_housing(), "top_head"),
        origin=Origin(xyz=(0.0, 0.143, 0.0)),
        material=satin_black,
        name="top_head",
    )

    # Feed slot on top center
    head.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.200, 0.018, 0.008), 0.005),
            "feed_slot",
        ),
        origin=Origin(xyz=(0.0, 0.143, 0.087)),
        material=gloss_black,
        name="feed_slot",
    )

    # Top recessed panel
    head.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.220, 0.140, 0.020), 0.004),
            "top_recess",
        ),
        origin=Origin(xyz=(0.0, 0.143, 0.084)),
        material=charcoal,
        name="top_recess",
    )

    # Control recess near rear
    head.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.120, 0.036, 0.010), 0.004),
            "control_recess",
        ),
        origin=Origin(xyz=(0.035, 0.060, 0.086)),
        material=gloss_black,
        name="control_recess",
    )

    # Shredder throat — front face opening
    head.visual(
        Box((0.160, 0.025, 0.022)),
        origin=Origin(xyz=(0.0, 0.265, 0.044)),
        material=gloss_black,
        name="shredder_throat",
    )

    # Cutter rollers (horizontal, along X axis)
    head.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.258, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="cutter_roller_0",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(0.0, 0.258, 0.054), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gloss_black,
        name="cutter_roller_1",
    )

    # Front badge (superellipse, flush on front face)
    head.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.100, 0.040, exponent=2.0), 0.004),
            "front_badge",
        ),
        origin=Origin(xyz=(0.0, 0.286, 0.050), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="front_badge",
    )

    # Indicator dots on top
    for i, x in enumerate([-0.060, -0.040, -0.020, 0.000, 0.020, 0.040, 0.060]):
        head.visual(
            Cylinder(radius=0.003, length=0.003),
            origin=Origin(xyz=(x, 0.143, 0.088)),
            material=pale_icon,
            name=f"indicator_dot_{i}",
        )

    # -- shell_to_head: REVOLUTE at the rear of the rim --
    rim_outer_r = 0.143  # 0.135 + 0.008
    bin_height = 0.300

    model.articulation(
        "shell_to_head",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=head,
        origin=Origin(xyz=(0.0, -rim_outer_r, bin_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.5, lower=0.0, upper=1.30,
        ),
    )

    # ------------------------------------------------------------------
    # BUTTONS (parented to head, on top near rear)
    # ------------------------------------------------------------------
    button_specs = [
        ("button_0", -0.010, 0.060),
        ("button_1", 0.016, 0.060),
    ]
    for bname, bx, by in button_specs:
        button = model.part(bname)
        button.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=pale_icon,
            name="button_cap",
        )
        model.articulation(
            f"head_to_{bname}",
            ArticulationType.PRISMATIC,
            parent=head,
            child=button,
            origin=Origin(xyz=(bx, by, 0.088)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.04, lower=0.0, upper=0.004,
            ),
        )

    # ------------------------------------------------------------------
    # CONTROL SWITCH (parented to head)
    # ------------------------------------------------------------------
    switch = model.part("control_switch")
    switch.visual(
        Box((0.032, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=pale_icon,
        name="slider_tab",
    )
    model.articulation(
        "head_to_switch",
        ArticulationType.PRISMATIC,
        parent=head,
        child=switch,
        origin=Origin(xyz=(0.053, 0.060, 0.088)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=0.06, lower=-0.022, upper=0.022,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    head = object_model.get_part("head")
    head_tilt = object_model.get_articulation("shell_to_head")
    switch_slide = object_model.get_articulation("head_to_switch")
    button_0_joint = object_model.get_articulation("head_to_button_0")

    # --- Rim-to-head seating (compatibility probe) ---
    ctx.expect_gap(
        head,
        shell,
        axis="z",
        positive_elem="top_head",
        negative_elem="rim_ring",
        max_gap=0.005,
        max_penetration=0.002,
        name="shell_to_head: head housing seats cleanly on round rim ring",
    )

    # Head disk covers the rim ring in plan view
    ctx.expect_overlap(
        head,
        shell,
        axes="xy",
        elem_a="top_head",
        elem_b="rim_ring",
        min_overlap=0.005,
        name="shell_to_head: head disk fully covers rim ring in plan",
    )

    # Feed slot spans the head top
    ctx.expect_overlap(
        head,
        head,
        axes="x",
        elem_a="feed_slot",
        elem_b="top_head",
        min_overlap=0.180,
        name="feed_slot spans the top_head panel",
    )

    # --- Tilt articulation proof ---
    # At rest the shredder throat is above the rim
    ctx.expect_gap(
        head,
        shell,
        axis="z",
        positive_elem="shredder_throat",
        negative_elem="rim_ring",
        min_gap=0.010,
        name="shredder_throat clears rim_ring at rest",
    )

    # At q=1.0 the throat lifts well above the rim
    with ctx.pose({head_tilt: 1.0}):
        ctx.expect_gap(
            head,
            shell,
            axis="z",
            positive_elem="shredder_throat",
            negative_elem="rim_ring",
            min_gap=0.15,
            name="shell_to_head tilted: throat lifts well above the rim",
        )

    # --- Button press still works on head ---
    rest_button = ctx.part_world_position(object_model.get_part("button_0"))
    with ctx.pose({button_0_joint: 0.004}):
        pressed_button = ctx.part_world_position(object_model.get_part("button_0"))
    ctx.check(
        "button_0 depresses downward on head",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    # --- Switch slide still works on head ---
    rest_switch = ctx.part_world_position(object_model.get_part("control_switch"))
    with ctx.pose({switch_slide: 0.022}):
        moved_switch = ctx.part_world_position(object_model.get_part("control_switch"))
    ctx.check(
        "control_switch slides across head panel",
        rest_switch is not None
        and moved_switch is not None
        and moved_switch[0] > rest_switch[0] + 0.015,
        details=f"rest={rest_switch}, moved={moved_switch}",
    )

    # --- Head tilt lifts the button upward (proves articulation direction) ---
    with ctx.pose({head_tilt: 0.0}):
        rest_btn_z = ctx.part_world_position(object_model.get_part("button_0"))
    with ctx.pose({head_tilt: 1.0}):
        tilt_btn_z = ctx.part_world_position(object_model.get_part("button_0"))
    ctx.check(
        "shell_to_head positive tilt raises controls upward",
        rest_btn_z is not None
        and tilt_btn_z is not None
        and tilt_btn_z[2] > rest_btn_z[2] + 0.005,
        details=f"rest_z={rest_btn_z[2]:.4f}, tilted_z={tilt_btn_z[2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
