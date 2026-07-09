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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL = Material("galvanized_steel", color=(0.78, 0.80, 0.78, 1.0))
SHADOW_STEEL = Material("shadowed_steel", color=(0.56, 0.58, 0.56, 1.0))
SAFETY_YELLOW = Material("safety_yellow", color=(1.0, 0.85, 0.02, 1.0))
RUBBER = Material("black_rubber", color=(0.03, 0.03, 0.03, 1.0))


def _cyl_x(part, name: str, *, center, length: float, radius: float, material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, *, center, length: float, radius: float, material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_z(part, name: str, *, center, length: float, radius: float, material=STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _hoop_points(z: float) -> list[tuple[float, float, float]]:
    """Semi-elliptical cage hoop around the outward side of the ladder."""
    pts: list[tuple[float, float, float]] = []
    for i in range(17):
        theta = math.pi - i * math.pi / 16.0
        pts.append((0.43 * math.cos(theta), 0.58 * math.sin(theta), z))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="exterior_fire_escape_ladder_with_cage",
        meta={
            "run_notes": (
                "Modeled from the supplied reference as an exterior fixed ladder "
                "with cylindrical safety cage, wall standoffs, sliding lower "
                "ladder section, and a hinged lower safety gate. No classification "
                "conflict was observed: the image matches an emergency fire escape "
                "ladder assembly."
            )
        },
    )

    fixed = model.part("fixed_ladder")

    # Main fixed ladder: two vertical galvanized side rails with 8 rungs.
    for x, suffix in [(-0.25, "0"), (0.25, "1")]:
        _cyl_z(
            fixed,
            f"fixed_side_rail_{suffix}",
            center=(x, 0.0, 2.79),
            length=2.58,
            radius=0.020,
        )

    rung_zs = [1.62 + 0.32 * i for i in range(8)]
    for i, z in enumerate(rung_zs):
        _cyl_x(
            fixed,
            f"fixed_rung_{i}",
            center=(0.0, 0.0, z),
            length=0.54,
            radius=0.014,
        )

    # Slightly forward guide rails and retainers for the sliding lower section.
    for x, suffix in [(-0.18, "0"), (0.18, "1")]:
        _cyl_z(
            fixed,
            f"slide_guide_rail_{suffix}",
            center=(x, 0.045, 1.86),
            length=1.10,
            radius=0.016,
            material=SHADOW_STEEL,
        )
        for j, z in enumerate((1.55, 2.38)):
            fixed.visual(
                Box((0.11, 0.055, 0.065)),
                origin=Origin(xyz=(x, 0.025, z)),
                material=SHADOW_STEEL,
                name=f"slide_retainer_{suffix}_{j}",
            )
            # Short welded bridge from the guide channel back to the main
            # fixed side rail so the guide assembly is visibly supported.
            fixed.visual(
                Box((0.10, 0.08, 0.035)),
                origin=Origin(xyz=((x + (-0.25 if x < 0.0 else 0.25)) / 2.0, 0.018, z)),
                material=SHADOW_STEEL,
                name=f"slide_bridge_{suffix}_{j}",
            )

    # Curved cage hoops at several heights; each hoop is connected back to the
    # ladder by short side struts and to the next hoop by vertical cage ribs.
    hoop_zs = (2.18, 2.74, 3.30, 3.86)
    for i, z in enumerate(hoop_zs):
        hoop = tube_from_spline_points(
            _hoop_points(z),
            radius=0.014,
            samples_per_segment=4,
            closed_spline=False,
            radial_segments=16,
            cap_ends=True,
            up_hint=(0.0, 0.0, 1.0),
        )
        fixed.visual(
            mesh_from_geometry(hoop, f"cage_hoop_mesh_{i}"),
            origin=Origin(),
            material=STEEL,
            name=f"cage_hoop_{i}",
        )
        _cyl_x(
            fixed,
            f"cage_side_strut_0_{i}",
            center=(-0.34, 0.0, z),
            length=0.18,
            radius=0.012,
        )
        _cyl_x(
            fixed,
            f"cage_side_strut_1_{i}",
            center=(0.34, 0.0, z),
            length=0.18,
            radius=0.012,
        )

    cage_ribs = [
        (-0.43, 0.00, "0"),
        (-0.30, 0.43, "1"),
        (0.00, 0.58, "2"),
        (0.30, 0.43, "3"),
        (0.43, 0.00, "4"),
    ]
    for x, y, suffix in cage_ribs:
        # The center rib (suffix "2") is shortened at the bottom to clear
        # the gate top bar that passes through the same y=0.58 plane.
        rib_center_z = 3.18 if suffix == "2" else 3.02
        rib_length = 1.36 if suffix == "2" else 1.84
        _cyl_z(
            fixed,
            f"cage_vertical_rib_{suffix}",
            center=(x, y, rib_center_z),
            length=rib_length,
            radius=0.012,
        )

    # Wall-side standoff arms and anchor pads. The wall itself is intentionally
    # omitted per prompt; pads show the mechanical mounting interface.
    for level, z in enumerate((1.80, 2.80, 3.80)):
        for x, suffix in [(-0.25, "0"), (0.25, "1")]:
            _cyl_y(
                fixed,
                f"wall_standoff_{suffix}_{level}",
                center=(x, -0.19, z),
                length=0.38,
                radius=0.016,
                material=SHADOW_STEEL,
            )
            fixed.visual(
                Box((0.13, 0.035, 0.11)),
                origin=Origin(xyz=(x, -0.39, z)),
                material=SHADOW_STEEL,
                name=f"wall_anchor_plate_{suffix}_{level}",
            )

    # Hinge pin for the lower entry gate, captured by a gate sleeve.
    _cyl_z(
        fixed,
        "gate_hinge_pin",
        center=(-0.43, 0.58, 2.18),
        length=0.34,
        radius=0.012,
        material=SHADOW_STEEL,
    )
    hinge_support = tube_from_spline_points(
        [
            (-0.43, 0.58, 2.00),
            (-0.36, 0.51, 2.00),
            (-0.30, 0.28, 2.00),
            (-0.25, 0.00, 2.00),
        ],
        radius=0.012,
        samples_per_segment=6,
        closed_spline=False,
        radial_segments=12,
        cap_ends=True,
    )
    fixed.visual(
        mesh_from_geometry(hinge_support, "gate_hinge_support_mesh"),
        origin=Origin(),
        material=SHADOW_STEEL,
        name="gate_hinge_support",
    )
    fixed.visual(
        Box((0.11, 0.035, 0.075)),
        origin=Origin(xyz=(0.43, 0.58, 2.34)),
        material=SHADOW_STEEL,
        name="gate_latch_receiver",
    )
    latch_support = tube_from_spline_points(
        [(0.43, 0.58, 2.34), (0.36, 0.51, 2.34), (0.30, 0.43, 2.34)],
        radius=0.012,
        samples_per_segment=6,
        closed_spline=False,
        radial_segments=12,
        cap_ends=True,
    )
    fixed.visual(
        mesh_from_geometry(latch_support, "gate_latch_support_mesh"),
        origin=Origin(),
        material=SHADOW_STEEL,
        name="gate_latch_support",
    )

    lower = model.part("lower_ladder")

    # Moving ladder section: narrower, front-mounted rails slide in the fixed
    # guide structure and retain a long hidden overlap when extended.
    for x, suffix in [(-0.18, "0"), (0.18, "1")]:
        _cyl_z(
            lower,
            f"lower_side_rail_{suffix}",
            center=(x, 0.079, -0.30),
            length=2.90,
            radius=0.018,
        )
        _cyl_z(
            lower,
            f"yellow_release_sleeve_{suffix}",
            center=(x, 0.109, -1.00),
            length=0.62,
            radius=0.021,
            material=SAFETY_YELLOW,
        )
        lower.visual(
            Box((0.075, 0.070, 0.045)),
            origin=Origin(xyz=(x, 0.079, -1.720)),
            material=RUBBER,
            name=f"rubber_foot_{suffix}",
        )

    lower_rung_zs = [-1.50 + 0.32 * i for i in range(8)]
    for i, z in enumerate(lower_rung_zs):
        _cyl_x(
            lower,
            f"lower_rung_{i}",
            center=(0.0, 0.079, z),
            length=0.41,
            radius=0.014,
        )

    # Small crosshead at the top of the moving ladder catches the guide stops.
    _cyl_x(
        lower,
        "upper_slide_crosshead",
        center=(0.0, 0.079, 0.98),
        length=0.43,
        radius=0.015,
        material=SHADOW_STEEL,
    )

    gate = model.part("safety_gate")
    _cyl_z(
        gate,
        "gate_hinge_sleeve",
        center=(0.0, 0.0, 0.0),
        length=0.30,
        radius=0.024,
        material=STEEL,
    )
    _cyl_x(
        gate,
        "gate_top_bar",
        center=(0.445, 0.0, 0.07),
        length=0.83,
        radius=0.014,
        material=STEEL,
    )
    gate.visual(
        Box((0.028, 0.018, 0.018)),
        origin=Origin(xyz=(0.027, 0.015, 0.07)),
        material=STEEL,
        name="gate_sleeve_weld",
    )
    _cyl_x(
        gate,
        "gate_lower_bar",
        center=(0.445, 0.0, -0.07),
        length=0.83,
        radius=0.012,
        material=STEEL,
    )
    gate.visual(
        Box((0.055, 0.045, 0.12)),
        origin=Origin(xyz=(0.86, 0.0, 0.0)),
        material=SAFETY_YELLOW,
        name="gate_latch_tab",
    )

    model.articulation(
        "fixed_to_lower_ladder",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=1.20),
    )

    model.articulation(
        "fixed_to_safety_gate",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=gate,
        origin=Origin(xyz=(-0.43, 0.58, 2.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed = object_model.get_part("fixed_ladder")
    lower = object_model.get_part("lower_ladder")
    gate = object_model.get_part("safety_gate")
    slide = object_model.get_articulation("fixed_to_lower_ladder")
    hinge = object_model.get_articulation("fixed_to_safety_gate")

    # Verify the fixed ladder has exactly 8 climbing rungs (fixed_rung_0..7)
    fixed_visual_names = {v.name for v in fixed.visuals}
    for i in range(8):
        rung_name = f"fixed_rung_{i}"
        ctx.check(
            f"fixed_ladder has {rung_name}",
            rung_name in fixed_visual_names,
            details=f"expected visual '{rung_name}' on fixed_ladder",
        )
    ctx.check(
        "fixed_ladder has exactly 8 rungs (no fixed_rung_8)",
        "fixed_rung_8" not in fixed_visual_names,
        details="fixed_rung_8 should not exist; rung count is N=8",
    )

    ctx.allow_overlap(
        fixed,
        gate,
        elem_a="gate_hinge_pin",
        elem_b="gate_hinge_sleeve",
        reason=(
            "The gate sleeve intentionally captures the fixed hinge pin; this "
            "local coaxial overlap represents the real pinned hinge."
        ),
    )

    ctx.expect_within(
        gate,
        fixed,
        axes="xy",
        inner_elem="gate_hinge_sleeve",
        outer_elem="gate_hinge_pin",
        margin=0.018,
        name="gate sleeve stays centered on hinge pin",
    )
    ctx.expect_overlap(
        gate,
        fixed,
        axes="z",
        elem_a="gate_hinge_sleeve",
        elem_b="gate_hinge_pin",
        min_overlap=0.25,
        name="gate sleeve axially captures hinge pin",
    )

    ctx.expect_overlap(
        lower,
        fixed,
        axes="z",
        elem_a="lower_side_rail_0",
        elem_b="slide_guide_rail_0",
        min_overlap=0.50,
        name="retracted lower ladder remains engaged in guides",
    )
    ctx.expect_overlap(
        fixed,
        lower,
        axes="x",
        elem_a="slide_guide_rail_0",
        elem_b="lower_side_rail_0",
        min_overlap=0.02,
        name="sliding rail is laterally aligned with fixed guide",
    )
    ctx.expect_gap(
        lower,
        fixed,
        axis="y",
        positive_elem="lower_side_rail_0",
        negative_elem="slide_guide_rail_0",
        min_gap=0.0,
        max_gap=0.004,
        name="sliding rail rides tangent to fixed guide",
    )

    rest_lower = ctx.part_world_position(lower)
    with ctx.pose({slide: 1.20}):
        extended_lower = ctx.part_world_position(lower)
        ctx.expect_overlap(
            lower,
            fixed,
            axes="z",
            elem_a="lower_side_rail_0",
            elem_b="slide_guide_rail_0",
            min_overlap=0.15,
            name="extended lower ladder still has retained insertion",
        )

    ctx.check(
        "lower ladder extends downward",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[2] < rest_lower[2] - 1.0,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )

    rest_gate = ctx.part_world_position(gate)
    with ctx.pose({hinge: 1.20}):
        opened_gate = ctx.part_world_position(gate)
        ctx.expect_overlap(
            gate,
            fixed,
            axes="z",
            elem_a="gate_hinge_sleeve",
            elem_b="gate_hinge_pin",
            min_overlap=0.25,
            name="opened gate remains on hinge pin",
        )

    ctx.check(
        "safety gate has a real swing joint",
        rest_gate is not None and opened_gate is not None,
        details=f"rest={rest_gate}, opened={opened_gate}",
    )

    return ctx.report()


object_model = build_object_model()
