from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_plate_mesh(name: str, length: float, width: float, thickness: float, radius: float):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=10),
            thickness,
            center=True,
        ),
    )


def _circle_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tube_mesh(name: str, outer_r: float, inner_r: float, height: float):
    """Hollow tube mesh via annular extrusion — reads as a real pipe/tube."""
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_r),
        [_circle_profile(inner_r)],
        height,
        center=True,
    )
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_laptop_stand")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    soft_black = model.material("soft_black", rgba=(0.055, 0.057, 0.060, 1.0))
    groove_black = model.material("groove_black", rgba=(0.0, 0.0, 0.0, 1.0))
    hardware = model.material("brushed_silver", rgba=(0.76, 0.76, 0.74, 1.0))
    laptop_silver = model.material("thin_laptop_silver", rgba=(0.80, 0.82, 0.83, 1.0))

    # ===== BASE =====
    base = model.part("base")
    base.visual(
        _rounded_plate_mesh("rounded_base_plate", 0.34, 0.24, 0.016, 0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=matte_black,
        name="base_plate",
    )
    for side, y in enumerate((-0.078, 0.078)):
        base.visual(
            Box((0.240, 0.006, 0.0022)),
            origin=Origin(xyz=(0.000, y, 0.0149)),
            material=groove_black,
            name=f"side_groove_{side}",
        )
    for side, x in enumerate((-0.126, 0.126)):
        base.visual(
            Box((0.006, 0.156, 0.0022)),
            origin=Origin(xyz=(x, 0.0, 0.0149)),
            material=groove_black,
            name=f"end_groove_{side}",
        )

    # ===== TURNTABLE =====
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.090, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=soft_black,
        name="outer_rotation_disk",
    )
    turntable.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=matte_black,
        name="upper_turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.046, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=matte_black,
        name="short_pedestal",
    )
    # Column-mount flange at the top of the pedestal
    turntable.visual(
        Cylinder(radius=0.042, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=matte_black,
        name="post_mount_flange",
    )

    # ===== OUTER POST (replaces link_arms) =====
    # Part origin at the column base; bolted to the turntable flange.
    outer_post = model.part("outer_post")
    outer_post.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=matte_black,
        name="base_collar",
    )
    # Hollow outer tube — inner radius 0.024 m accepts the inner slider tube.
    outer_post.visual(
        _tube_mesh("outer_tube_shell", 0.030, 0.024, 0.192),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=matte_black,
        name="outer_tube",
    )
    # Height-release lever paddle on the side of the column.
    outer_post.visual(
        Box((0.024, 0.010, 0.014)),
        origin=Origin(xyz=(0.038, 0.0, 0.170)),
        material=soft_black,
        name="release_lever",
    )

    # ===== INNER SLIDER =====
    # Part origin at the lift-column joint frame (top of outer tube at q=0).
    # Inner tube extends downward into the outer tube and upward above it.
    inner_slider = model.part("inner_slider")
    inner_slider.visual(
        _tube_mesh("inner_tube_shell", 0.023, 0.018, 0.240),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=matte_black,
        name="inner_tube",
    )
    # Mounting plate at the top of the inner slider for the tilt pivot.
    inner_slider.visual(
        Box((0.064, 0.064, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=matte_black,
        name="top_mount_plate",
    )
    # Two pivot lugs carry the tray tilt shaft.
    for side, y in enumerate((-0.028, 0.028)):
        inner_slider.visual(
            Box((0.022, 0.012, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.082)),
            material=matte_black,
            name=f"pivot_lug_{side}",
        )
    # Silver pivot shaft through the lugs.
    inner_slider.visual(
        Cylinder(radius=0.007, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.082), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_shaft",
    )

    # ===== UPPER TRAY =====
    tray = model.part("upper_tray")
    # Pivot collar wraps around the shaft between the lugs.
    tray.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_collar",
    )
    # Undermount plate bridges the pivot collar to the tray rails above.
    tray.visual(
        Box((0.056, 0.050, 0.016)),
        origin=Origin(xyz=(0.012, 0.0, 0.020)),
        material=matte_black,
        name="tray_undermount",
    )
    tray.visual(
        Box((0.082, 0.230, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.032)),
        material=matte_black,
        name="front_tray_rail",
    )
    for side, y in enumerate((-0.092, 0.092)):
        tray.visual(
            Box((0.300, 0.018, 0.014)),
            origin=Origin(xyz=(-0.110, y, 0.032)),
            material=matte_black,
            name=f"side_tray_rail_{side}",
        )
    # Thin laptop-shaped plate for load context only.
    tray.visual(
        _rounded_plate_mesh("thin_laptop_plate", 0.300, 0.215, 0.006, 0.014),
        origin=Origin(xyz=(-0.110, 0.0, 0.042)),
        material=laptop_silver,
        name="laptop_plate",
    )
    for side, y in enumerate((-0.066, 0.066)):
        tray.visual(
            Box((0.022, 0.058, 0.066)),
            origin=Origin(xyz=(0.056, y, 0.072)),
            material=soft_black,
            name=f"front_lip_{side}",
        )
        tray.visual(
            Box((0.034, 0.064, 0.014)),
            origin=Origin(xyz=(0.044, y, 0.044)),
            material=soft_black,
            name=f"lip_foot_{side}",
        )

    # ===== ARTICULATIONS =====
    model.articulation(
        "turntable_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0),
    )
    model.articulation(
        "turntable_to_post",
        ArticulationType.FIXED,
        parent=turntable,
        child=outer_post,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )
    model.articulation(
        "lift_column",
        ArticulationType.PRISMATIC,
        parent=outer_post,
        child=inner_slider,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.15, lower=0.0, upper=0.100),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_slider,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.35, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    outer_post = object_model.get_part("outer_post")
    inner_slider = object_model.get_part("inner_slider")
    tray = object_model.get_part("upper_tray")

    # --- Telescoping tube nesting ---
    # The inner tube is intentionally captured inside the outer tube bore.
    ctx.allow_overlap(
        outer_post,
        inner_slider,
        elem_a="outer_tube",
        elem_b="inner_tube",
        reason="The inner telescoping tube is intentionally nested inside the outer tube bore for prismatic height adjustment.",
    )

    # --- Pivot shaft captured through tray collar ---
    ctx.allow_overlap(
        inner_slider,
        tray,
        elem_a="pivot_shaft",
        elem_b="pivot_collar",
        reason="The silver pivot shaft is intentionally captured through the tray pivot collar for tilt adjustment.",
    )

    # --- Structural presence checks ---
    ctx.check(
        "broad rounded base plate present",
        base.get_visual("base_plate") is not None
        and len([v for v in base.visuals if "groove" in (v.name or "")]) >= 4,
        details="Expected rounded base plate and shallow groove visuals.",
    )
    ctx.check(
        "turntable disk and pedestal present",
        turntable.get_visual("upper_turntable_disk") is not None
        and turntable.get_visual("short_pedestal") is not None,
        details="Expected central rotating disk/pedestal.",
    )
    ctx.check(
        "outer_post tube present",
        outer_post.get_visual("outer_tube") is not None,
        details="Expected outer telescoping column tube.",
    )
    ctx.check(
        "inner_slider tube present",
        inner_slider.get_visual("inner_tube") is not None,
        details="Expected inner telescoping column tube.",
    )
    ctx.check(
        "front retaining lips present",
        tray.get_visual("front_lip_0") is not None
        and tray.get_visual("front_lip_1") is not None,
        details="Expected two front clamps/lips.",
    )

    # --- Joint graph ---
    nonfixed = [
        j
        for j in object_model.articulations
        if str(j.articulation_type).split(".")[-1] != "FIXED"
    ]
    ctx.check(
        "turntable_yaw, lift_column, and tray_tilt joints present",
        len(nonfixed) >= 3
        and object_model.get_articulation("turntable_yaw") is not None
        and object_model.get_articulation("lift_column") is not None
        and object_model.get_articulation("tray_tilt") is not None,
        details="Expected continuous yaw, prismatic lift column, and revolute tray tilt.",
    )

    # Verify the lift_column is specifically a PRISMATIC joint on the vertical axis.
    lift_art = object_model.get_articulation("lift_column")
    ctx.check(
        "lift_column is prismatic on Z axis",
        str(lift_art.articulation_type).split(".")[-1] == "PRISMATIC"
        and abs(lift_art.axis[2]) > 0.9,
        details=f"type={lift_art.articulation_type}, axis={lift_art.axis}",
    )

    # --- Telescoping nesting proof (inner tube inside outer tube) ---
    ctx.expect_within(
        inner_slider,
        outer_post,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.002,
        name="inner tube centered in outer tube on XY",
    )
    ctx.expect_overlap(
        inner_slider,
        outer_post,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.060,
        name="inner tube retains insertion in outer tube at rest",
    )

    # --- Turntable seating ---
    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        name="turntable sits on base",
    )

    # --- Lift column vertical travel ---
    lift_joint = object_model.get_articulation("lift_column")
    tray_joint = object_model.get_articulation("tray_tilt")
    rest_pos = ctx.part_world_position(tray)

    with ctx.pose({lift_joint: 0.100}):
        ctx.expect_overlap(
            inner_slider,
            outer_post,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.060,
            name="inner tube retains insertion at max lift extension",
        )
        extended_pos = ctx.part_world_position(tray)

    ctx.check(
        "lift_column raises tray vertically",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.080,
        details=f"rest_z={rest_pos}, extended_z={extended_pos}",
    )

    # --- Tilt motion ---
    rest_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({tray_joint: 0.40}):
        posed_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "tray_tilt tilts the tray",
        rest_aabb is not None
        and posed_aabb is not None
        and abs(posed_aabb[1][2] - rest_aabb[1][2]) > 0.005,
        details=f"rest_max_z={rest_aabb[1][2]}, posed_max_z={posed_aabb[1][2]}",
    )

    # --- Pivot shaft capture proof ---
    ctx.expect_overlap(
        inner_slider,
        tray,
        axes="yz",
        elem_a="pivot_shaft",
        elem_b="pivot_collar",
        min_overlap=0.010,
        name="pivot shaft captured through tray collar",
    )

    return ctx.report()


object_model = build_object_model()
