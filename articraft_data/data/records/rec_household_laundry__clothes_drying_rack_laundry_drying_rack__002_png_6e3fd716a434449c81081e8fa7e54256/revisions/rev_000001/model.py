from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


WHITE_METAL = Material("white_powder_coated_metal", rgba=(0.92, 0.94, 0.92, 1.0))
BRIGHT_BAR = Material("slightly_worn_white_bar", rgba=(0.86, 0.88, 0.86, 1.0))
BLACK_PLASTIC = Material("black_plastic_hinges_and_feet", rgba=(0.015, 0.014, 0.013, 1.0))
DARK_GREY = Material("dark_grey_rubber", rgba=(0.07, 0.08, 0.08, 1.0))
PALE_GREY = Material("pale_grey_end_caps", rgba=(0.70, 0.72, 0.72, 1.0))


def _origin_for_bar(p1: tuple[float, float, float], p2: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that aligns a local-Z cylinder between two points."""

    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1.0e-9:
        raise ValueError("bar endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5), rpy=(0.0, pitch, yaw)), length


def _bar(part, name: str, p1, p2, radius: float, material=WHITE_METAL) -> None:
    origin, length = _origin_for_bar(p1, p2)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _sphere(part, name: str, xyz, radius: float, material=BLACK_PLASTIC) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def _interp(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def _add_hanger_geometry(part, prefix: str, material=BLACK_PLASTIC) -> None:
    """A small empty plastic hanger whose part frame lies on the drying bar axis."""

    wire = 0.004
    # C-shaped hook around (but not intersecting) the parent drying bar.
    center = (0.0, 0.0, -0.010)
    hook_r = 0.033
    angles = [math.radians(a) for a in (150, 112, 74, 38, 2, -34, -70)]
    points = [(center[0] + hook_r * math.cos(a), 0.0, center[2] + hook_r * math.sin(a)) for a in angles]
    for i in range(len(points) - 1):
        _bar(part, f"{prefix}_hook_{i}", points[i], points[i + 1], wire, material)

    neck_top = points[-1]
    neck_bottom = (0.010, 0.0, -0.090)
    _bar(part, f"{prefix}_neck", neck_top, neck_bottom, wire, material)

    left_shoulder = (-0.125, 0.0, -0.185)
    right_shoulder = (0.125, 0.0, -0.185)
    bottom_left = (-0.095, 0.0, -0.245)
    bottom_right = (0.095, 0.0, -0.245)
    _bar(part, f"{prefix}_left_arm", neck_bottom, left_shoulder, wire, material)
    _bar(part, f"{prefix}_right_arm", neck_bottom, right_shoulder, wire, material)
    _bar(part, f"{prefix}_lower_left", left_shoulder, bottom_left, wire, material)
    _bar(part, f"{prefix}_lower_bar", bottom_left, bottom_right, wire, material)
    _bar(part, f"{prefix}_lower_right", bottom_right, right_shoulder, wire, material)
    _sphere(part, f"{prefix}_crown", neck_bottom, 0.008, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="freestanding_laundry_drying_rack",
        meta={
            "reference_note": (
                "Modeled the visible rack core only: tubular metal frame, hinged wings, "
                "folding support legs, braces, plastic feet/caps, and a few empty hangers; no laundry is included."
            )
        },
    )

    # The central spine is the fixed root carrying the hinge line for the wings
    # and the folding A-frame legs.
    spine = model.part("spine")
    _bar(spine, "central_hinge_tube", (0.0, -0.42, 0.86), (0.0, 0.42, 0.86), 0.018, BRIGHT_BAR)
    _bar(spine, "lower_lock_tube", (0.0, -0.37, 0.54), (0.0, 0.37, 0.54), 0.010, PALE_GREY)
    for side, y in (("side_0", -0.43), ("side_1", 0.43)):
        _sphere(spine, f"{side}_pivot_knuckle", (0.0, y, 0.86), 0.045, BLACK_PLASTIC)
        _sphere(spine, f"{side}_lower_lock", (0.0, y * 0.86, 0.54), 0.024, BLACK_PLASTIC)
        _bar(spine, f"{side}_short_vertical_socket", (0.0, y, 0.81), (0.0, y, 0.91), 0.012, BLACK_PLASTIC)
        _bar(spine, f"{side}_lock_strut", (0.0, y, 0.84), (0.0, y * 0.86, 0.56), 0.006, BLACK_PLASTIC)

    # Folding A-frame leg assemblies.  Their local origins are on the central
    # hinge tube, so q=0 is the deployed pose and the limits allow folding.
    low_legs = model.part("low_leg_frame")
    high_legs = model.part("high_leg_frame")
    leg_r = 0.014
    brace_r = 0.007
    low_feet = [(-0.68, -0.50, -0.84), (-0.68, 0.50, -0.84)]
    high_feet = [(0.76, -0.50, -0.84), (0.76, 0.50, -0.84)]
    for idx, y in enumerate((-0.36, 0.36)):
        low_top = (-0.025, y, -0.015)
        high_top = (0.025, y, -0.015)
        low_foot = low_feet[idx]
        high_foot = high_feet[idx]
        _bar(low_legs, f"leg_{idx}", low_top, low_foot, leg_r, WHITE_METAL)
        _bar(high_legs, f"leg_{idx}", high_top, high_foot, leg_r, WHITE_METAL)
        _bar(low_legs, f"side_cross_brace_{idx}", (-0.075, y, -0.080), (-0.585, -0.50 if y > 0 else 0.50, -0.720), brace_r, WHITE_METAL)
        _bar(high_legs, f"side_cross_brace_{idx}", (0.090, y, -0.080), (0.655, -0.50 if y > 0 else 0.50, -0.720), brace_r, WHITE_METAL)
        low_collar = (0.0, y * 0.80, 0.0)
        high_collar = (0.0, y * 1.12, 0.0)
        _sphere(low_legs, f"pivot_collar_{idx}", low_collar, 0.024, BLACK_PLASTIC)
        _sphere(high_legs, f"pivot_collar_{idx}", high_collar, 0.024, BLACK_PLASTIC)
        _bar(low_legs, f"pivot_web_{idx}", low_collar, low_top, 0.010, BLACK_PLASTIC)
        _bar(high_legs, f"pivot_web_{idx}", high_collar, high_top, 0.010, BLACK_PLASTIC)
        _sphere(low_legs, f"foot_{idx}", low_foot, 0.034, BLACK_PLASTIC)
        _sphere(high_legs, f"foot_{idx}", high_foot, 0.034, BLACK_PLASTIC)
        _bar(low_legs, f"rubber_sole_{idx}", (low_foot[0] - 0.045, low_foot[1], low_foot[2] - 0.010), (low_foot[0] + 0.045, low_foot[1], low_foot[2] - 0.010), 0.013, DARK_GREY)
        _bar(high_legs, f"rubber_sole_{idx}", (high_foot[0] - 0.045, high_foot[1], high_foot[2] - 0.010), (high_foot[0] + 0.045, high_foot[1], high_foot[2] - 0.010), 0.013, DARK_GREY)
    _bar(low_legs, "front_floor_crossbar", low_feet[0], low_feet[1], 0.011, PALE_GREY)
    _bar(low_legs, "lower_shelf_bar", (-0.44, -0.45, -0.54), (-0.44, 0.45, -0.54), 0.010, PALE_GREY)
    _bar(high_legs, "rear_floor_crossbar", high_feet[0], high_feet[1], 0.011, PALE_GREY)
    _bar(high_legs, "lower_shelf_bar", (0.49, -0.45, -0.55), (0.49, 0.45, -0.55), 0.010, PALE_GREY)

    # Low, almost-horizontal drying wing.
    low_wing = model.part("low_wing")
    rail_r = 0.013
    rung_r = 0.010
    low_start_left = (-0.055, -0.34, -0.010)
    low_start_right = (-0.055, 0.34, -0.010)
    low_end_left = (-1.18, -0.34, -0.090)
    low_end_right = (-1.18, 0.34, -0.090)
    _bar(low_wing, "side_rail_0", low_start_left, low_end_left, rail_r, WHITE_METAL)
    _bar(low_wing, "side_rail_1", low_start_right, low_end_right, rail_r, WHITE_METAL)
    for i, t in enumerate((0.05, 0.20, 0.35, 0.50, 0.65, 0.80, 0.96)):
        p0 = _interp(low_start_left, low_end_left, t)
        p1 = _interp(low_start_right, low_end_right, t)
        _bar(low_wing, f"drying_bar_{i}", p0, p1, rung_r, BRIGHT_BAR)
        _sphere(low_wing, f"bar_socket_{i}_0", p0, 0.017, BLACK_PLASTIC)
        _sphere(low_wing, f"bar_socket_{i}_1", p1, 0.017, BLACK_PLASTIC)
    _bar(low_wing, "outer_handle", low_end_left, low_end_right, 0.016, BRIGHT_BAR)
    for side, p in (("0", low_end_left), ("1", low_end_right)):
        _bar(low_wing, f"outer_black_cap_{side}", (p[0] - 0.035, p[1], p[2]), (p[0] + 0.035, p[1], p[2]), 0.023, BLACK_PLASTIC)
    for side, y in (("0", -0.34), ("1", 0.34)):
        _bar(low_wing, f"hinge_socket_{side}", (-0.065, y, -0.035), (0.020, y, 0.020), 0.020, BLACK_PLASTIC)
        _bar(low_wing, f"folding_link_{side}", (-0.42, y * 0.92, -0.19), (-0.08, y, -0.010), 0.006, WHITE_METAL)

    # Raised rear/tall drying wing, echoing the stepped high side frame in the reference.
    high_wing = model.part("high_wing")
    high_start_left = (0.055, -0.34, 0.010)
    high_start_right = (0.055, 0.34, 0.010)
    high_end_left = (0.86, -0.34, 0.64)
    high_end_right = (0.86, 0.34, 0.64)
    _bar(high_wing, "side_rail_0", high_start_left, high_end_left, rail_r, WHITE_METAL)
    _bar(high_wing, "side_rail_1", high_start_right, high_end_right, rail_r, WHITE_METAL)
    for i, t in enumerate((0.06, 0.22, 0.38, 0.54, 0.70, 0.86, 1.00)):
        p0 = _interp(high_start_left, high_end_left, t)
        p1 = _interp(high_start_right, high_end_right, t)
        _bar(high_wing, f"drying_bar_{i}", p0, p1, rung_r, BRIGHT_BAR)
        _sphere(high_wing, f"bar_socket_{i}_0", p0, 0.017, BLACK_PLASTIC)
        _sphere(high_wing, f"bar_socket_{i}_1", p1, 0.017, BLACK_PLASTIC)
    _bar(high_wing, "top_handle", high_end_left, high_end_right, 0.016, BRIGHT_BAR)
    for side, p in (("0", high_end_left), ("1", high_end_right)):
        _bar(high_wing, f"top_black_cap_{side}", (p[0] - 0.040, p[1], p[2]), (p[0] + 0.040, p[1], p[2]), 0.024, BLACK_PLASTIC)
    for side, y in (("0", -0.34), ("1", 0.34)):
        y_out = y * 1.18
        _bar(high_wing, f"hinge_socket_{side}", (-0.020, y_out, -0.010), (0.075, y_out, 0.045), 0.020, BLACK_PLASTIC)
        _bar(high_wing, f"hinge_web_{side}", (0.070, y_out, 0.040), high_start_left if y < 0 else high_start_right, 0.009, BLACK_PLASTIC)
        _bar(high_wing, f"folding_link_{side}", (0.10, y, 0.06), (0.54, y * 0.92, -0.29), 0.006, WHITE_METAL)

    # A few optional empty hangers are included as articulated parts.  They pivot
    # about the bar axis with only a modest swing range, as real hangers do.
    hanger_specs = [
        ("hanger_0", low_wing, (-0.97, -0.16, -0.075), "low_wing_to_hanger_0"),
        ("hanger_1", low_wing, (-0.97, -0.04, -0.075), "low_wing_to_hanger_1"),
        ("hanger_2", high_wing, (0.74, 0.05, 0.545), "high_wing_to_hanger_2"),
        ("hanger_3", high_wing, (0.74, 0.22, 0.545), "high_wing_to_hanger_3"),
    ]
    for hanger_name, parent, local_xyz, joint_name in hanger_specs:
        hanger = model.part(hanger_name)
        _add_hanger_geometry(hanger, hanger_name)
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=hanger,
            origin=Origin(xyz=local_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=1.5, lower=-0.35, upper=0.35),
        )

    model.articulation(
        "spine_to_low_legs",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=low_legs,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.30, upper=0.65),
    )
    model.articulation(
        "spine_to_high_legs",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=high_legs,
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.30, upper=0.65),
    )
    model.articulation(
        "spine_to_low_wing",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=low_wing,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.35, upper=0.90),
    )
    model.articulation(
        "spine_to_high_wing",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=high_wing,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.75, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spine = object_model.get_part("spine")
    low_wing = object_model.get_part("low_wing")
    high_wing = object_model.get_part("high_wing")
    low_legs = object_model.get_part("low_leg_frame")
    high_legs = object_model.get_part("high_leg_frame")
    low_hinge = object_model.get_articulation("spine_to_low_wing")
    high_hinge = object_model.get_articulation("spine_to_high_wing")

    root_names = [p.name for p in object_model.root_parts()]
    ctx.check("single fixed spine root", root_names == ["spine"], details=f"roots={root_names}")

    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no clothing parts modeled",
        all(token not in name for name in part_names for token in ("cloth", "shirt", "towel", "sock")),
        details=f"parts={part_names}",
    )
    ctx.check(
        "several articulated empty hangers",
        len([name for name in part_names if name.startswith("hanger_")]) == 4,
        details=f"parts={part_names}",
    )

    aabb = ctx.part_world_aabb(spine)
    low_aabb = ctx.part_world_aabb(low_wing)
    high_aabb = ctx.part_world_aabb(high_wing)
    ctx.check("center hinge is narrow transverse tube", aabb is not None and (aabb[1][1] - aabb[0][1]) > 0.75)
    ctx.check("low wing projects outward for drying bars", low_aabb is not None and low_aabb[0][0] < -1.15)
    ctx.check("raised wing reaches high drying tier", high_aabb is not None and high_aabb[1][2] > 1.45)

    ctx.expect_overlap(low_wing, spine, axes="y", min_overlap=0.55, name="low wing shares hinge span with spine")
    ctx.expect_overlap(high_wing, spine, axes="y", min_overlap=0.55, name="high wing shares hinge span with spine")
    ctx.expect_overlap(low_legs, spine, axes="y", min_overlap=0.65, name="low legs straddle hinge width")
    ctx.expect_overlap(high_legs, spine, axes="y", min_overlap=0.65, name="high legs straddle hinge width")
    ctx.expect_overlap(
        "hanger_2",
        high_wing,
        axes="xz",
        min_overlap=0.004,
        name="hanger 2 hook captures its drying bar",
    )
    ctx.expect_overlap(
        "hanger_3",
        high_wing,
        axes="xz",
        min_overlap=0.004,
        name="hanger 3 hook captures its drying bar",
    )
    ctx.expect_overlap(
        high_legs,
        high_wing,
        axes="xyz",
        min_overlap=0.015,
        name="high leg collar is seated in high wing hinge socket",
    )
    ctx.expect_overlap(
        low_legs,
        low_wing,
        axes="xyz",
        min_overlap=0.015,
        name="low leg pivot web is seated in low wing hinge socket",
    )

    before_low = ctx.part_world_aabb(low_wing)
    before_high = ctx.part_world_aabb(high_wing)
    with ctx.pose({low_hinge: 0.45, high_hinge: 0.25}):
        after_low = ctx.part_world_aabb(low_wing)
        after_high = ctx.part_world_aabb(high_wing)
    ctx.check(
        "hinged wings change pose about central bar",
        before_low is not None
        and after_low is not None
        and before_high is not None
        and after_high is not None
        and abs(after_low[0][2] - before_low[0][2]) > 0.05
        and abs(after_high[1][2] - before_high[1][2]) > 0.03,
        details=f"low_before={before_low}, low_after={after_low}, high_before={before_high}, high_after={after_high}",
    )

    # Captured hinge/socket visual overlaps are small, local, and intentional:
    # the simplified black plastic knuckles seat around the shared metal hinge line.
    for child in ("low_leg_frame", "high_leg_frame", "low_wing", "high_wing"):
        ctx.allow_overlap(
            "spine",
            child,
            reason="Local hinge knuckle/socket seating around the central folding rack pivot.",
        )
    ctx.allow_overlap(
        "hanger_2",
        "high_wing",
        reason="The hanger hook is intentionally captured around the drying bar with slight seated contact.",
    )
    ctx.allow_overlap(
        "hanger_3",
        "high_wing",
        reason="The hanger hook is intentionally captured around the drying bar with slight seated contact.",
    )
    ctx.allow_overlap(
        "high_leg_frame",
        "high_wing",
        reason="Stacked plastic hinge collar is seated in the matching wing hinge socket.",
    )
    ctx.allow_overlap(
        "low_leg_frame",
        "low_wing",
        reason="The molded pivot web is intentionally nested in the wing hinge socket.",
    )

    return ctx.report()


object_model = build_object_model()
