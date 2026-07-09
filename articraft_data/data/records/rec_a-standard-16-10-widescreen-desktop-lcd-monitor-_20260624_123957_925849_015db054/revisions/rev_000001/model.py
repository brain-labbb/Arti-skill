from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_HEIGHT = 0.026
PILLAR_HEIGHT = 0.340
HINGE_Z = 0.435
HINGE_Y = 0.026

SCREEN_OUTER_W = 0.585
SCREEN_OUTER_H = 0.375
PANEL_W = 0.512
PANEL_H = 0.320
SCREEN_CENTER_Z = 0.105


def _hex_base_solid() -> cq.Workplane:
    """Low, angular six-sided monitor foot, authored in meters."""
    points = [
        (-0.195, -0.115),
        (-0.145, -0.155),
        (0.145, -0.155),
        (0.195, -0.115),
        (0.170, 0.125),
        (-0.170, 0.125),
    ]
    return cq.Workplane("XY").polyline(points).close().extrude(BASE_HEIGHT)


def _slotted_pillar_solid() -> cq.Workplane:
    """Central pillar with a rounded cable-management cut-out through it."""
    width = 0.082
    depth = 0.056
    y_center = 0.040
    height = PILLAR_HEIGHT
    z_center = BASE_HEIGHT + height / 2.0

    pillar = cq.Workplane("XY").box(width, depth, height).translate((0.0, y_center, z_center))

    slot_w = 0.034
    slot_h = 0.150
    slot_center_z = BASE_HEIGHT + 0.195
    slot_straight_h = slot_h - slot_w
    cut_depth = 0.125

    slot_rect = cq.Workplane("XY").box(slot_w, cut_depth, slot_straight_h).translate(
        (0.0, y_center, slot_center_z)
    )
    top_round = (
        cq.Workplane("XY")
        .cylinder(cut_depth, slot_w / 2.0)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, y_center, slot_center_z + slot_straight_h / 2.0))
    )
    bottom_round = (
        cq.Workplane("XY")
        .cylinder(cut_depth, slot_w / 2.0)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, y_center, slot_center_z - slot_straight_h / 2.0))
    )
    cutter = slot_rect.union(top_round).union(bottom_round)
    return pillar.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widescreen_lcd_monitor")

    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.007, 1.0))
    soft_black = model.material("soft_black", rgba=(0.018, 0.020, 0.020, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.060, 0.062, 0.060, 1.0))
    lcd_white = model.material("lcd_white", rgba=(0.88, 0.92, 0.95, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.38, 0.38, 0.35, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_hex_base_solid(), "hex_base"),
        material=soft_black,
        name="hex_base",
    )
    stand.visual(
        mesh_from_cadquery(_slotted_pillar_solid(), "slotted_pillar"),
        material=matte_black,
        name="slotted_pillar",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(0.0, 0.040, BASE_HEIGHT + 0.003)),
        material=dark_silver,
        name="base_collar",
    )
    stand.visual(
        Box((0.126, 0.052, 0.030)),
        origin=Origin(xyz=(0.0, 0.055, BASE_HEIGHT + PILLAR_HEIGHT + 0.010)),
        material=matte_black,
        name="top_crosshead",
    )

    for i, x in enumerate((-0.053, 0.053)):
        stand.visual(
            Box((0.026, 0.034, 0.070)),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z - 0.020)),
            material=matte_black,
            name=f"yoke_arm_{i}",
        )
        stand.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=matte_black,
            name=f"hinge_cheek_{i}",
        )

    screen = model.part("screen")
    screen.visual(
        Box((0.568, 0.034, 0.360)),
        origin=Origin(xyz=(0.0, -0.047, SCREEN_CENTER_Z)),
        material=soft_black,
        name="rear_shell",
    )
    screen.visual(
        mesh_from_geometry(
            BezelGeometry(
                (PANEL_W, PANEL_H),
                (SCREEN_OUTER_W, SCREEN_OUTER_H),
                0.020,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.018,
            ),
            "screen_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.067, SCREEN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bezel",
    )
    screen.visual(
        Box((PANEL_W, 0.003, PANEL_H)),
        origin=Origin(xyz=(0.0, -0.076, SCREEN_CENTER_Z)),
        material=lcd_white,
        name="display_panel",
    )
    screen.visual(
        Box((0.074, 0.042, 0.100)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark_graphite,
        name="rear_mount",
    )
    screen.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )

    model.articulation(
        "stand_to_screen",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=screen,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(-0.12, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.5, lower=-0.22, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    screen = object_model.get_part("screen")
    tilt = object_model.get_articulation("stand_to_screen")

    ctx.expect_gap(
        screen,
        stand,
        axis="z",
        min_gap=0.25,
        positive_elem="display_panel",
        negative_elem="hex_base",
        name="display sits well above the desktop base",
    )
    ctx.expect_overlap(
        screen,
        stand,
        axes="x",
        min_overlap=0.04,
        elem_a="hinge_barrel",
        elem_b="top_crosshead",
        name="screen hinge is centered over the stand head",
    )
    ctx.check(
        "panel is 16:10 widescreen",
        abs((PANEL_W / PANEL_H) - 1.6) < 0.02,
        details=f"panel ratio={PANEL_W / PANEL_H:.3f}",
    )

    with ctx.pose({tilt: -0.20}):
        back_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    with ctx.pose({tilt: 0.18}):
        forward_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    back_y = _center_y(back_aabb)
    forward_y = _center_y(forward_aabb)
    ctx.check(
        "tilt joint moves screen forward and back",
        back_y is not None and forward_y is not None and forward_y < back_y - 0.025,
        details=f"back_y={back_y}, forward_y={forward_y}",
    )

    return ctx.report()


object_model = build_object_model()
