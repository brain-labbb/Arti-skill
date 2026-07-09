from __future__ import annotations

import math

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


def _central_spine_mesh(name: str, length: float, width: float, thickness: float, rise: float, run: float):
    """Single central column oriented along the arm direction.

    The spine runs from local (0, 0, 0) at the base pivot to
    (run, 0, rise) at the tray pivot.  ``width`` is the visible face
    dimension perpendicular to the spine axis in the tilt plane and
    ``thickness`` is the side-to-side (Y) dimension.
    """
    geom = ExtrudeGeometry(
        rounded_rect_profile(length, width, min(length, width) * 0.12, corner_segments=6),
        thickness,
        center=True,
    )
    # Extrude direction (Z) → rotate to Y (side-to-side)
    geom.rotate_x(math.pi / 2.0)
    # Tilt so the spine length axis points from (0,0,0) toward (run,0,rise)
    theta = math.atan2(rise, run)
    geom.rotate_y(-theta)
    # Center between the two pivot points
    geom.translate(run * 0.5, 0.0, rise * 0.5)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_laptop_stand")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    soft_black = model.material("soft_black", rgba=(0.055, 0.057, 0.060, 1.0))
    groove_black = model.material("groove_black", rgba=(0.0, 0.0, 0.0, 1.0))
    hardware = model.material("brushed_silver", rgba=(0.76, 0.76, 0.74, 1.0))
    laptop_silver = model.material("thin_laptop_silver", rgba=(0.80, 0.82, 0.83, 1.0))

    # ── BASE ──────────────────────────────────────────────────────────────
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

    # ── TURNTABLE ─────────────────────────────────────────────────────────
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
    # Single central yoke bracket on the centerline (replaces paired base_lug_0/1).
    # A solid upright that captures the lower end of the central spine.
    turntable.visual(
        Box((0.044, 0.056, 0.072)),
        origin=Origin(xyz=(0.030, 0.0, 0.056)),
        material=matte_black,
        name="base_yoke",
    )
    # Silver pivot bushing through the yoke (visible on both sides).
    turntable.visual(
        Cylinder(radius=0.018, length=0.062),
        origin=Origin(xyz=(0.030, 0.0, 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="base_yoke_bushing",
    )

    # ── CENTRAL COLUMN (single spine replacing paired side arms) ──────────
    run = 0.160
    rise = 0.200
    link_length = math.sqrt(run * run + rise * rise)

    column = model.part("link_column")
    # Main structural spine — one central load path from base pivot to tray pivot.
    column.visual(
        _central_spine_mesh(
            "central_spine",
            length=link_length,
            width=0.050,
            thickness=0.032,
            rise=rise,
            run=run,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="central_spine",
    )


    # ── UPPER TRAY ────────────────────────────────────────────────────────
    tray = model.part("upper_tray")
    # Single central tray bracket hanging below the tray (replaces paired tray_lug_0/1).
    tray.visual(
        Box((0.042, 0.052, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_black,
        name="tray_bracket",
    )
    # Silver pivot bushing through the tray bracket.
    tray.visual(
        Cylinder(radius=0.018, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="tray_bracket_bushing",
    )
    # Tray front rail.
    tray.visual(
        Box((0.082, 0.230, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.032)),
        material=matte_black,
        name="front_tray_rail",
    )
    # Tray side rails.
    for side, y in enumerate((-0.092, 0.092)):
        tray.visual(
            Box((0.300, 0.018, 0.014)),
            origin=Origin(xyz=(-0.110, y, 0.032)),
            material=matte_black,
            name=f"side_tray_rail_{side}",
        )
    # Thin laptop-shaped plate — load context only; no screen or graphics.
    tray.visual(
        _rounded_plate_mesh("thin_laptop_plate", 0.300, 0.215, 0.006, 0.014),
        origin=Origin(xyz=(-0.110, 0.0, 0.042)),
        material=laptop_silver,
        name="laptop_plate",
    )
    # Front retaining lips and feet.
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

    # ── ARTICULATIONS ─────────────────────────────────────────────────────
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
        "arm_pitch",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=column,
        origin=Origin(xyz=(0.030, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=column,
        child=tray,
        origin=Origin(xyz=(run, 0.0, rise)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=-0.35, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    column = object_model.get_part("link_column")
    tray = object_model.get_part("upper_tray")

    # ── Intentional overlap allowances ────────────────────────────────────
    # The central spine lower end pivots inside the single base yoke bracket.
    ctx.allow_overlap(
        turntable, column,
        elem_a="base_yoke", elem_b="central_spine",
        reason="The central spine lower end pivots inside the single central base yoke bracket.",
    )
    ctx.expect_overlap(
        turntable, column,
        axes="yz",
        elem_a="base_yoke", elem_b="central_spine",
        min_overlap=0.010,
        name="base_yoke captures central spine at lower pivot",
    )

    # The silver bushing passes through the spine at the base pivot.
    ctx.allow_overlap(
        turntable, column,
        elem_a="base_yoke_bushing", elem_b="central_spine",
        reason="The silver pivot bushing passes through the central spine at the base pivot.",
    )
    ctx.expect_overlap(
        turntable, column,
        axes="yz",
        elem_a="base_yoke_bushing", elem_b="central_spine",
        min_overlap=0.010,
        name="base pivot bushing passes through central spine",
    )

    # The central spine upper end pivots inside the single tray bracket.
    ctx.allow_overlap(
        tray, column,
        elem_a="tray_bracket", elem_b="central_spine",
        reason="The central spine upper end pivots inside the single central tray bracket.",
    )
    ctx.expect_overlap(
        tray, column,
        axes="yz",
        elem_a="tray_bracket", elem_b="central_spine",
        min_overlap=0.010,
        name="tray_bracket captures central spine at upper pivot",
    )

    # The silver bushing passes through the spine at the tray pivot.
    ctx.allow_overlap(
        tray, column,
        elem_a="tray_bracket_bushing", elem_b="central_spine",
        reason="The silver pivot bushing passes through the central spine at the tray pivot.",
    )
    ctx.expect_overlap(
        tray, column,
        axes="yz",
        elem_a="tray_bracket_bushing", elem_b="central_spine",
        min_overlap=0.010,
        name="tray pivot bushing passes through central spine",
    )

    # ── Structural presence checks ───────────────────────────────────────
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
        details="Expected central rotating disk and pedestal.",
    )
    ctx.check(
        "single central spine column present on centerline",
        column.get_visual("central_spine") is not None,
        details="Expected single central load column replacing paired side arms.",
    )
    ctx.check(
        "central base yoke with pivot bushing",
        turntable.get_visual("base_yoke") is not None
        and turntable.get_visual("base_yoke_bushing") is not None,
        details="Expected single central yoke bracket with silver pivot bushing.",
    )
    ctx.check(
        "central tray bracket with pivot bushing",
        tray.get_visual("tray_bracket") is not None
        and tray.get_visual("tray_bracket_bushing") is not None,
        details="Expected single central tray bracket with silver pivot bushing.",
    )
    ctx.check(
        "front retaining lips present",
        tray.get_visual("front_lip_0") is not None
        and tray.get_visual("front_lip_1") is not None,
        details="Expected two front clamps/lips on the tray.",
    )

    nonfixed = [
        j for j in object_model.articulations
        if str(j.articulation_type).split(".")[-1] != "FIXED"
    ]
    ctx.check(
        "rotation and tilt joints present on single spine",
        len(nonfixed) >= 2
        and object_model.get_articulation("turntable_yaw") is not None
        and object_model.get_articulation("arm_pitch") is not None
        and object_model.get_articulation("tray_tilt") is not None,
        details="Expected yaw, arm_pitch, and tray_tilt articulations on the single spine.",
    )

    # ── Clearance and support checks ─────────────────────────────────────
    ctx.expect_gap(
        turntable, base, axis="z",
        max_gap=0.001, max_penetration=0.001,
        name="turntable sits on base",
    )
    ctx.expect_overlap(
        tray, column, axes="xy",
        min_overlap=0.018,
        elem_a="front_tray_rail", elem_b="central_spine",
        name="upper tray is carried by central spine",
    )
    # Prove the spine is on the centerline (single load path, not paired arms).
    spine_center = ctx.part_world_position(column)
    ctx.check(
        "central spine on Y centerline",
        spine_center is not None and abs(spine_center[1]) < 0.010,
        details=f"spine world position Y should be near 0, got {spine_center}",
    )

    # ── Articulation pose check ──────────────────────────────────────────
    arm_joint = object_model.get_articulation("arm_pitch")
    tray_joint = object_model.get_articulation("tray_tilt")
    rest_aabb = ctx.part_world_aabb(tray)
    with ctx.pose({arm_joint: 0.30, tray_joint: -0.20}):
        posed_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "adjustment joints move upper tray",
        rest_aabb is not None and posed_aabb is not None
        and abs(posed_aabb[0][2] - rest_aabb[0][2]) > 0.010,
        details=f"rest={rest_aabb}, posed={posed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
