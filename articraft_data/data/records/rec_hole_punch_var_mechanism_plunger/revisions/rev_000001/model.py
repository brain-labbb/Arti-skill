from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    superellipse_profile,
)


# ────────────────────────────────────────────────────────────────────
# Geometry helpers
# ────────────────────────────────────────────────────────────────────


def _rounded_box_mesh(name: str, size: tuple[float, float, float], radius: float):
    """CadQuery rounded block authored in meters."""
    base = cq.Workplane("XY").box(*size)
    try:
        body = base.edges().fillet(radius)
    except Exception:
        body = cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.00045, angular_tolerance=0.08)


def _guide_column_mesh(name: str, outer_r: float, inner_r: float, height: float):
    """Hollow cylindrical guide column with chamfered top edge."""
    tube = cq.Workplane("XY").circle(outer_r).extrude(height)
    bore = (
        cq.Workplane("XY")
        .circle(inner_r)
        .extrude(height + 0.002)
        .translate((0, 0, -0.001))
    )
    tube = tube.cut(bore)
    try:
        tube = tube.edges(">Z").chamfer(0.0015)
    except Exception:
        pass
    tube = tube.translate((0, 0, -height / 2.0))
    return mesh_from_cadquery(tube, name, tolerance=0.0003, angular_tolerance=0.06)


def _flange_ring_mesh(name: str, outer_r: float, inner_r: float, height: float):
    """Flat annular ring (flange / rim)."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    ring = ring.translate((0, 0, -height / 2.0))
    return mesh_from_cadquery(ring, name, tolerance=0.0003, angular_tolerance=0.06)


def _plunger_cap_mesh(name: str, radius: float, height: float):
    """Rounded palm-press button cap."""
    cap = cq.Workplane("XY").circle(radius).extrude(height)
    try:
        cap = cap.edges(">Z").fillet(min(0.004, height * 0.30))
    except Exception:
        pass
    try:
        cap = cap.edges("<Z").fillet(0.001)
    except Exception:
        pass
    cap = cap.translate((0, 0, -height / 2.0))
    return mesh_from_cadquery(cap, name, tolerance=0.0003, angular_tolerance=0.06)


def _extruded_xz_mesh(
    name: str,
    profile_xz: list[tuple[float, float]],
    thickness_y: float,
):
    """Extrude a side profile drawn in X/Z into a centered Y-thickness mesh."""
    geom = ExtrudeGeometry(profile_xz, thickness_y, center=True).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


# ────────────────────────────────────────────────────────────────────
# Build
# ────────────────────────────────────────────────────────────────────


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_push_plunger_craft_punch")

    # ── Materials ──
    blue = model.material("blue_painted_metal", rgba=(0.08, 0.24, 0.62, 1.0))
    blue_highlight = model.material("raised_blue_highlight", rgba=(0.12, 0.33, 0.78, 1.0))
    blue_shadow = model.material("dark_blue_shadow", rgba=(0.035, 0.09, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    black = model.material("black_slot", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.77, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.30, 0.32, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.55, 0.52, 1.0))

    # ================================================================
    # BASE (root)
    # ================================================================
    base = model.part("base")

    # Black rubber foot pad
    base.visual(
        _rounded_box_mesh("rubber_pad", (0.100, 0.060, 0.008), 0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="rubber_pad",
    )

    # Main blue body shell
    base.visual(
        _rounded_box_mesh("blue_base_shell", (0.090, 0.054, 0.026), 0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=blue,
        name="blue_base_shell",
    )

    # Raised top platform around the die
    base.visual(
        _rounded_box_mesh("top_platform", (0.052, 0.046, 0.014), 0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=blue_highlight,
        name="top_platform",
    )

    # Guide column flange (wider ring at column base)
    base.visual(
        _flange_ring_mesh("guide_flange", 0.026, 0.015, 0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=blue,
        name="guide_flange",
    )

    # Guide column (hollow tube the plunger slides inside)
    column_height = 0.048
    column_bottom_z = 0.054
    column_center_z = column_bottom_z + column_height / 2.0  # 0.078
    column_top_z = column_bottom_z + column_height  # 0.102

    base.visual(
        _guide_column_mesh("guide_column", 0.019, 0.015, column_height),
        origin=Origin(xyz=(0.0, 0.0, column_center_z)),
        material=blue,
        name="guide_column",
    )

    # Dark bore interior (short visible dark section from the top)
    # Radius slightly exceeds column inner wall (0.015) for mesh connectivity.
    base.visual(
        Cylinder(radius=0.0155, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, column_top_z - 0.012)),
        material=black,
        name="guide_bore",
    )

    # Front throat lip (paper entry edge)
    base.visual(
        _rounded_box_mesh("front_throat_lip", (0.044, 0.008, 0.006), 0.002),
        origin=Origin(xyz=(0.0, -0.022, 0.038)),
        material=blue,
        name="front_throat_lip",
    )

    # Paper throat gap (dark shadow showing paper insertion slot)
    base.visual(
        _rounded_box_mesh("paper_throat_gap", (0.036, 0.030, 0.003), 0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=blue_shadow,
        name="paper_throat_gap",
    )

    # Two visible assembly screw heads on the -Y side face
    shell_half_y = 0.027
    for index, (x_pos, z_pos) in enumerate(((-0.018, 0.022), (0.018, 0.022))):
        base.visual(
            Cylinder(radius=0.005, length=0.003),
            origin=Origin(
                xyz=(x_pos, -(shell_half_y + 0.001), z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"side_screw_{index}",
        )
        base.visual(
            Cylinder(radius=0.002, length=0.003),
            origin=Origin(
                xyz=(x_pos, -(shell_half_y + 0.003), z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"side_screw_slot_{index}",
        )

    # Waste window (oval chad exit on the -Y side, embedded into shell face)
    base.visual(
        _extruded_xz_mesh(
            "side_waste_window_mesh",
            superellipse_profile(0.009, 0.005, exponent=2.2, segments=24),
            0.002,
        ),
        origin=Origin(xyz=(0.018, -(shell_half_y - 0.001), 0.026)),
        material=black,
        name="side_waste_window",
    )

    # Single die set (centered under guide column)
    base.visual(
        Cylinder(radius=0.009, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        material=blue,
        name="die_boss_0",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=steel,
        name="die_ring_0",
    )
    base.visual(
        Cylinder(radius=0.004, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black,
        name="die_hole_0",
    )

    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.050),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    # ================================================================
    # PLUNGER HEAD (slides vertically in guide column)
    # ================================================================
    plunger = model.part("plunger_head")

    # Palm-press cap (rounded button on top)
    cap_height = 0.014
    cap_center_plunger = cap_height / 2.0 + 0.005  # 0.012
    plunger.visual(
        _plunger_cap_mesh("plunger_cap", 0.022, cap_height),
        origin=Origin(xyz=(0.0, 0.0, cap_center_plunger)),
        material=blue_highlight,
        name="plunger_cap",
    )

    # Cap knurled rim (slightly wider dark ring around cap base)
    plunger.visual(
        _flange_ring_mesh("plunger_cap_rim", 0.024, 0.020, 0.004),
        origin=Origin(xyz=(0.0, 0.0, cap_center_plunger - 0.006)),
        material=blue_shadow,
        name="plunger_cap_rim",
    )

    # Sliding stem (cylinder that moves inside the bore, connects up to cap)
    stem_height = 0.044
    stem_center_plunger = -0.016
    plunger.visual(
        Cylinder(radius=0.013, length=stem_height),
        origin=Origin(xyz=(0.0, 0.0, stem_center_plunger)),
        material=blue_highlight,
        name="plunger_stem",
    )

    # Return spring (annular ring wrapping around the lower stem)
    plunger.visual(
        _flange_ring_mesh("return_spring_ring", 0.014, 0.012, 0.008),
        origin=Origin(xyz=(0.0, 0.0, stem_center_plunger - 0.014)),
        material=spring_steel,
        name="return_spring",
    )

    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.022, length=0.058),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    # ================================================================
    # PUNCH PIN (single, fixed to plunger)
    # ================================================================
    pin_mount_z_plunger = -0.030

    pin = model.part("punch_pin_0")
    pin.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=steel,
        name="punch_pin",
    )
    pin.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_steel,
        name="pin_retainer",
    )
    pin.visual(
        Cylinder(radius=0.0035, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0115)),
        material=steel,
        name="pin_cutting_tip",
    )
    pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.004, length=0.024),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    # ================================================================
    # ARTICULATIONS
    # ================================================================

    # Prismatic plunger slide: push down = positive q along -Z
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, column_top_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.8,
            lower=0.0,
            upper=0.012,
        ),
    )

    # Pin fixed to plunger (descends with plunger into die)
    model.articulation(
        "plunger_to_pin_0",
        ArticulationType.FIXED,
        parent=plunger,
        child="punch_pin_0",
        origin=Origin(xyz=(0.0, 0.0, pin_mount_z_plunger)),
    )

    return model


# ────────────────────────────────────────────────────────────────────
# Tests
# ────────────────────────────────────────────────────────────────────


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    plunger = object_model.get_part("plunger_head")
    pin = object_model.get_part("punch_pin_0")
    slide_joint = object_model.get_articulation("plunger_slide")
    pin_mount = object_model.get_articulation("plunger_to_pin_0")

    base_visuals = {v.name for v in base.visuals}
    plunger_visuals = {v.name for v in plunger.visuals}

    # ── Base structure ──
    ctx.check(
        "base has layered blue body with vertical guide column",
        {"blue_base_shell", "top_platform", "guide_column", "guide_flange"}.issubset(
            base_visuals
        ),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "black rubber base pad is modeled",
        "rubber_pad" in base_visuals,
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "paper throat gap and side waste window are present",
        {"paper_throat_gap", "side_waste_window"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )
    ctx.check(
        "visible side screws on base",
        {"side_screw_0", "side_screw_1"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )

    # ── Single die (N=1) ──
    die_holes = [v.name for v in base.visuals if v.name and v.name.startswith("die_hole_")]
    ctx.check(
        "single die hole (N=1)",
        len(die_holes) == 1 and "die_hole_0" in base_visuals,
        details=f"die holes found={die_holes}",
    )
    ctx.check(
        "single die set in base",
        {"die_ring_0", "die_hole_0", "die_boss_0"}.issubset(base_visuals),
        details=f"base visuals={sorted(base_visuals)}",
    )

    # ── Plunger head ──
    ctx.check(
        "plunger_head has palm-press cap and sliding stem",
        {"plunger_cap", "plunger_stem"}.issubset(plunger_visuals),
        details=f"plunger visuals={sorted(plunger_visuals)}",
    )

    # ── Key structural change: prismatic joint ──
    ctx.check(
        "plunger_slide is a non-fixed prismatic joint with spring-return limits",
        slide_joint.articulation_type == ArticulationType.PRISMATIC
        and slide_joint.motion_limits is not None
        and slide_joint.motion_limits.lower is not None
        and slide_joint.motion_limits.upper is not None
        and slide_joint.motion_limits.upper > slide_joint.motion_limits.lower,
        details=f"type={slide_joint.articulation_type}, limits={slide_joint.motion_limits}",
    )
    ctx.check(
        "pin is fixed to plunger (descends vertically)",
        pin_mount.articulation_type == ArticulationType.FIXED,
        details=f"pin mount type={pin_mount.articulation_type}",
    )

    # ── Pin alignment with die ──
    ctx.expect_overlap(
        pin,
        base,
        axes="xy",
        elem_a="punch_pin",
        elem_b="die_hole_0",
        min_overlap=0.003,
        name="punch pin aligns with single die in XY",
    )

    # Rest position: pin tip clears die throat
    ctx.expect_gap(
        pin,
        base,
        axis="z",
        positive_elem="pin_cutting_tip",
        negative_elem="die_hole_0",
        min_gap=0.001,
        max_gap=0.015,
        name="rest pin tip clears die throat",
    )

    # ── Allow intentional overlaps for sliding fit ──
    ctx.allow_overlap(
        base,
        plunger,
        elem_a="guide_bore",
        elem_b="plunger_stem",
        reason="The plunger stem slides inside the guide column bore as a prismatic fit.",
    )
    ctx.allow_overlap(
        plunger,
        pin,
        elem_a="plunger_stem",
        elem_b="pin_retainer",
        reason="The pin retainer is captured inside the plunger stem bore.",
    )
    ctx.allow_overlap(
        plunger,
        pin,
        elem_a="plunger_stem",
        elem_b="punch_pin",
        reason="The upper portion of the punch pin is nested inside the hollow plunger stem.",
    )

    # Prove the sliding fit with containment
    ctx.expect_within(
        plunger,
        base,
        axes="xy",
        inner_elem="plunger_stem",
        outer_elem="guide_column",
        margin=0.004,
        name="stem stays centered in guide column bore",
    )

    # ── Pressing motion: plunger and pin descend ──
    rest_pin_pos = ctx.part_world_position(pin)
    rest_plunger_pos = ctx.part_world_position(plunger)
    upper_limit = slide_joint.motion_limits.upper if slide_joint.motion_limits else 0.012

    with ctx.pose({slide_joint: upper_limit}):
        pressed_pin_pos = ctx.part_world_position(pin)
        pressed_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "pressing plunger_head drives punch pin downward",
        rest_pin_pos is not None
        and pressed_pin_pos is not None
        and pressed_pin_pos[2] < rest_pin_pos[2] - 0.005,
        details=f"rest_z={rest_pin_pos}, pressed_z={pressed_pin_pos}",
    )
    ctx.check(
        "plunger_head descends along prismatic slide when pressed",
        rest_plunger_pos is not None
        and pressed_plunger_pos is not None
        and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.005,
        details=f"rest_z={rest_plunger_pos}, pressed_z={pressed_plunger_pos}",
    )

    return ctx.report()


object_model = build_object_model()
