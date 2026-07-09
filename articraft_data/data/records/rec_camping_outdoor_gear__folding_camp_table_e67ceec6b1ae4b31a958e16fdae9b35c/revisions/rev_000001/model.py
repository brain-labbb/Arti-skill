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
    PerforatedPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(part, name: str, p0, p1, radius: float, material: Material) -> None:
    """Add a cylinder whose local Z axis runs between two local points."""

    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux) if abs(ux) + abs(uy) > 1e-9 else 0.0
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_leg_sleeve(root, idx: int, x: float, y: float, mat: Material) -> None:
    root.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(x, y, 0.55)),
        material=mat,
        name=f"leg_sleeve_{idx}",
    )
    root.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(x, y, 0.405)),
        material=mat,
        name=f"lower_collar_{idx}",
    )
    root.visual(
        Cylinder(radius=0.025, length=0.036),
        origin=Origin(xyz=(x, y, 0.620)),
        material=mat,
        name=f"upper_collar_{idx}",
    )
    # Small adjustable clamp handle seated in the collar.
    knob_side = 1.0 if y > 0.0 else -1.0
    root.visual(
        Box((0.042, 0.075, 0.020)),
        origin=Origin(xyz=(x, y + knob_side * 0.045, 0.455)),
        material=mat,
        name=f"leg_clamp_{idx}",
    )


def _add_lower_leg(model, root, idx: int, x: float, y: float, mat: Material):
    leg = model.part(f"lower_leg_{idx}")
    leg.visual(
        Cylinder(radius=0.012, length=0.45),
        # Top of the inner tube is intentionally retained inside the parent
        # sleeve at z=+0.08 in the child frame.
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=mat,
        name=f"inner_tube_{idx}",
    )
    leg.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=mat,
        name=f"height_lock_{idx}",
    )
    knob_side = 1.0 if y > 0.0 else -1.0
    leg.visual(
        Box((0.038, 0.055, 0.020)),
        origin=Origin(xyz=(0.0, knob_side * 0.026, -0.080)),
        material=mat,
        name=f"thumb_knob_{idx}",
    )
    leg.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.371)),
        material=mat,
        name=f"foot_pad_{idx}",
    )
    model.articulation(
        f"table_to_lower_leg_{idx}",
        ArticulationType.PRISMATIC,
        parent=root,
        child=leg,
        origin=Origin(xyz=(x, y, 0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.16),
    )
    return leg


def _add_brace(
    model,
    root,
    name: str,
    hinge_xyz,
    end_xyz,
    y_offset: float,
    mat: Material,
):
    hx, hy, hz = hinge_xyz
    ex, ey, ez = end_xyz
    brace = model.part(name)
    local_end = (ex - hx, ey - hy, ez - hz)
    _cylinder_between(
        brace,
        "tube",
        (0.0, 0.0, 0.0),
        local_end,
        0.007,
        mat,
    )
    brace.visual(Sphere(radius=0.014), origin=Origin(), material=mat, name="hinge_eye")
    brace.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=local_end),
        material=mat,
        name="end_eye",
    )
    model.articulation(
        f"table_to_{name}",
        ArticulationType.REVOLUTE,
        parent=root,
        child=brace,
        origin=Origin(xyz=(hx, hy + y_offset, hz)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=0.0, upper=0.75),
    )
    return brace


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="lightweight_folding_camp_table",
        meta={
            "run_notes": (
                "Primary reference reads as a lightweight folding camp table; "
                "no classification conflict with the category name was suspected."
            )
        },
    )

    graphite = model.material("graphite_slats", rgba=(0.035, 0.037, 0.036, 1.0))
    rail_black = model.material("satin_black_tubes", rgba=(0.005, 0.005, 0.005, 1.0))
    dark_fabric = model.material("black_fabric", rgba=(0.015, 0.013, 0.012, 1.0))
    mesh_gray = model.material("gray_mesh", rgba=(0.18, 0.18, 0.17, 0.48))
    orange = model.material("orange_binding", rgba=(0.95, 0.25, 0.045, 1.0))
    logo_white = model.material("small_white_logo", rgba=(0.92, 0.92, 0.88, 1.0))

    table = model.part("tabletop_frame")

    # Roll-top style slatted rectangular tabletop.
    slat_count = 13
    slat_w = 0.038
    gap = 0.008
    start_y = -((slat_count - 1) * (slat_w + gap)) * 0.5
    for i in range(slat_count):
        y = start_y + i * (slat_w + gap)
        table.visual(
            Box((1.06, slat_w, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.720)),
            material=graphite,
            name=f"table_slat_{i}",
        )

    # Black perimeter and underside rails mechanically tie the slats together.
    table.visual(Box((1.105, 0.034, 0.030)), origin=Origin(xyz=(0.0, -0.303, 0.704)), material=rail_black, name="front_rail")
    table.visual(Box((1.105, 0.034, 0.030)), origin=Origin(xyz=(0.0, 0.303, 0.704)), material=rail_black, name="rear_rail")
    table.visual(Box((0.040, 0.650, 0.030)), origin=Origin(xyz=(-0.532, 0.0, 0.704)), material=rail_black, name="end_rail_0")
    table.visual(Box((0.040, 0.650, 0.030)), origin=Origin(xyz=(0.532, 0.0, 0.704)), material=rail_black, name="end_rail_1")
    for x in (-0.32, 0.0, 0.32):
        table.visual(
            Box((0.024, 0.650, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.684)),
            material=rail_black,
            name=f"under_crossbar_{x:+.2f}",
        )

    # Small white mark on one edge to echo the reference logo without importing
    # marketing styling.
    table.visual(
        Box((0.085, 0.020, 0.0035)),
        origin=Origin(xyz=(-0.485, -0.282, 0.7295)),
        material=logo_white,
        name="small_edge_mark",
    )

    # Four upper leg sleeves, collars, and clamp knobs are fixed to the frame.
    leg_xy = [(-0.480, -0.255), (-0.480, 0.255), (0.480, -0.255), (0.480, 0.255)]
    for idx, (x, y) in enumerate(leg_xy):
        _add_leg_sleeve(table, idx, x, y, rail_black)

    # Perimeter tubes just below the tabletop and lower side spreader rails.
    _cylinder_between(table, "front_tube", (-0.500, -0.278, 0.630), (0.500, -0.278, 0.630), 0.010, rail_black)
    _cylinder_between(table, "rear_tube", (-0.500, 0.278, 0.630), (0.500, 0.278, 0.630), 0.010, rail_black)
    _cylinder_between(table, "left_tube", (-0.505, -0.260, 0.530), (-0.505, 0.260, 0.530), 0.009, rail_black)
    _cylinder_between(table, "right_tube", (0.505, -0.260, 0.530), (0.505, 0.260, 0.530), 0.009, rail_black)
    _cylinder_between(table, "front_lower_spreader", (-0.500, -0.278, 0.395), (0.500, -0.278, 0.395), 0.010, rail_black)
    _cylinder_between(table, "rear_lower_spreader", (-0.500, 0.278, 0.395), (0.500, 0.278, 0.395), 0.010, rail_black)

    # Under-table fabric storage pocket with orange bound edges and mesh sides.
    table.visual(
        Box((0.820, 0.490, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=dark_fabric,
        name="pocket_bottom",
    )
    mesh_panel = PerforatedPanelGeometry(
        (0.820, 0.165),
        0.003,
        hole_diameter=0.006,
        pitch=0.014,
        frame=0.010,
        center=True,
    )
    table.visual(
        mesh_from_geometry(mesh_panel, "front_mesh_panel"),
        origin=Origin(xyz=(0.0, -0.237, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mesh_gray,
        name="front_mesh_panel",
    )
    table.visual(
        mesh_from_geometry(mesh_panel, "rear_mesh_panel"),
        origin=Origin(xyz=(0.0, 0.237, 0.430), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mesh_gray,
        name="rear_mesh_panel",
    )
    table.visual(Box((0.820, 0.018, 0.020)), origin=Origin(xyz=(0.0, -0.244, 0.510)), material=orange, name="front_top_binding")
    table.visual(Box((0.820, 0.018, 0.020)), origin=Origin(xyz=(0.0, 0.244, 0.510)), material=orange, name="rear_top_binding")
    table.visual(Box((0.840, 0.018, 0.020)), origin=Origin(xyz=(0.0, -0.244, 0.352)), material=orange, name="front_bottom_binding")
    table.visual(Box((0.840, 0.018, 0.020)), origin=Origin(xyz=(0.0, 0.244, 0.352)), material=orange, name="rear_bottom_binding")
    table.visual(Box((0.020, 0.490, 0.170)), origin=Origin(xyz=(-0.420, 0.0, 0.430)), material=dark_fabric, name="pocket_end_0")
    table.visual(Box((0.020, 0.490, 0.170)), origin=Origin(xyz=(0.420, 0.0, 0.430)), material=dark_fabric, name="pocket_end_1")
    for x in (-0.360, 0.360):
        for y in (-0.225, 0.225):
            table.visual(
                Box((0.020, 0.020, 0.145)),
                origin=Origin(xyz=(x, y, 0.585)),
                material=orange,
                name=f"hanger_{x:+.2f}_{y:+.2f}",
            )
            tube_y = -0.278 if y < 0.0 else 0.278
            table.visual(
                Box((0.022, 0.070, 0.020)),
                origin=Origin(xyz=(x, (y + tube_y) * 0.5, 0.630)),
                material=orange,
                name=f"suspender_{x:+.2f}_{y:+.2f}",
            )

    # Telescoping lower leg parts.
    for idx, (x, y) in enumerate(leg_xy):
        _add_lower_leg(model, table, idx, x, y, rail_black)

    # Hinged X/scissor braces on front and rear faces.  The two bars on each
    # face are slightly offset through depth so they visibly cross without a
    # false 3D collision.
    _add_brace(model, table, "front_brace_0", (-0.410, -0.298, 0.405), (0.410, -0.298, 0.610), 0.0, rail_black)
    _add_brace(model, table, "front_brace_1", (-0.410, -0.256, 0.610), (0.410, -0.256, 0.405), 0.0, rail_black)
    _add_brace(model, table, "rear_brace_0", (-0.410, 0.256, 0.405), (0.410, 0.256, 0.610), 0.0, rail_black)
    _add_brace(model, table, "rear_brace_1", (-0.410, 0.298, 0.610), (0.410, 0.298, 0.405), 0.0, rail_black)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    table = object_model.get_part("tabletop_frame")

    # Intentional retained insertion: lower leg tubes slide inside simplified
    # solid upper sleeve proxies.
    for idx in range(4):
        leg = object_model.get_part(f"lower_leg_{idx}")
        joint = object_model.get_articulation(f"table_to_lower_leg_{idx}")
        ctx.allow_overlap(
            table,
            leg,
            elem_a=f"leg_sleeve_{idx}",
            elem_b=f"inner_tube_{idx}",
            reason="The telescoping inner leg is intentionally retained inside the upper sleeve proxy.",
        )
        ctx.allow_overlap(
            table,
            leg,
            elem_a=f"lower_collar_{idx}",
            elem_b=f"inner_tube_{idx}",
            reason="The adjustable collar encircles the sliding leg tube at the sleeve mouth.",
        )
        ctx.expect_within(
            leg,
            table,
            axes="xy",
            inner_elem=f"inner_tube_{idx}",
            outer_elem=f"leg_sleeve_{idx}",
            margin=0.003,
            name=f"lower leg {idx} centered in sleeve",
        )
        ctx.expect_overlap(
            leg,
            table,
            axes="z",
            elem_a=f"inner_tube_{idx}",
            elem_b=f"leg_sleeve_{idx}",
            min_overlap=0.07,
            name=f"lower leg {idx} retained in sleeve",
        )
        rest = ctx.part_world_position(leg)
        with ctx.pose({joint: 0.16}):
            ctx.expect_within(
                leg,
                table,
                axes="xy",
                inner_elem=f"inner_tube_{idx}",
                outer_elem=f"leg_sleeve_{idx}",
                margin=0.003,
                name=f"retracted lower leg {idx} stays centered",
            )
            high = ctx.part_world_position(leg)
        ctx.check(
            f"lower leg {idx} retracts upward",
            rest is not None and high is not None and high[2] > rest[2] + 0.12,
            details=f"rest={rest}, retracted={high}",
        )

    # Prompt-specific geometry checks: slatted table top footprint, hanging
    # pocket, and brace motion.
    ctx.expect_overlap("tabletop_frame", "tabletop_frame", axes="xy", elem_a="table_slat_0", elem_b="front_rail", min_overlap=0.008, name="slats captured by perimeter rail")
    ctx.expect_gap("tabletop_frame", "tabletop_frame", axis="z", positive_elem="table_slat_6", negative_elem="front_top_binding", min_gap=0.18, name="fabric pocket hangs below tabletop")

    brace = object_model.get_part("front_brace_0")
    brace_joint = object_model.get_articulation("table_to_front_brace_0")
    rest = ctx.part_world_aabb(brace)
    with ctx.pose({brace_joint: 0.55}):
        folded = ctx.part_world_aabb(brace)
    ctx.check(
        "front scissor brace hinges",
        rest is not None
        and folded is not None
        and abs(folded[1][2] - rest[1][2]) > 0.05,
        details=f"rest={rest}, folded={folded}",
    )

    return ctx.report()


object_model = build_object_model()
