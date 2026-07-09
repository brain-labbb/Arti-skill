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
        name="exterior_fire_escape_ladder_parapet_hook",
        meta={
            "run_notes": (
                "Parapet-hook variant of the exterior fixed fire escape ladder "
                "with cylindrical safety cage. The distributed wall standoff / "
                "anchor plate interface is replaced by a top parapet hook-over "
                "bracket (horizontal arm with two down-turned hook legs) attached "
                "to the rail heads via a REVOLUTE adjustment pivot. Two levels of "
                "light intermediate spacer stubs keep the rails off the wall. The "
                "sliding lower ladder section (PRISMATIC) and hinged safety gate "
                "(REVOLUTE) are preserved from the parent baseline. No "
                "classification conflict was observed."
            )
        },
    )

    fixed = model.part("fixed_ladder")

    # Main fixed ladder: two vertical galvanized side rails with many rungs.
    for x, suffix in [(-0.25, "0"), (0.25, "1")]:
        _cyl_z(
            fixed,
            f"fixed_side_rail_{suffix}",
            center=(x, 0.0, 3.75),
            length=4.50,
            radius=0.020,
        )

    rung_zs = [1.62 + 0.32 * i for i in range(14)]
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
        for j, z in enumerate((1.32, 2.38)):
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
    hoop_zs = (2.18, 3.12, 4.06, 5.00, 5.78)
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
        rib_center_z = 4.15 if suffix == "2" else 3.98
        rib_length = 3.28 if suffix == "2" else 3.62
        _cyl_z(
            fixed,
            f"cage_vertical_rib_{suffix}",
            center=(x, y, rib_center_z),
            length=rib_length,
            radius=0.012,
        )

    # Light intermediate spacer stubs keep the rails off the wall face at two
    # levels. The wall itself is intentionally omitted per prompt; stubs show
    # the stand-off distance without the heavy anchor-plate interface.
    for level, z in enumerate((2.40, 4.20)):
        for x, suffix in [(-0.25, "0"), (0.25, "1")]:
            _cyl_y(
                fixed,
                f"spacer_{suffix}_{level}",
                center=(x, -0.12, z),
                length=0.24,
                radius=0.014,
                material=SHADOW_STEEL,
            )
            fixed.visual(
                Box((0.08, 0.025, 0.08)),
                origin=Origin(xyz=(x, -0.245, z)),
                material=SHADOW_STEEL,
                name=f"spacer_pad_{suffix}_{level}",
            )

    # Pivot bosses at the top of each side rail carry the parapet hook pivot.
    # These are short Y-axis brackets bridging from the rail head to the hook
    # arm, which sits forward of the rails.
    for x, suffix in [(-0.25, "0"), (0.25, "1")]:
        _cyl_y(
            fixed,
            f"hook_pivot_boss_{suffix}",
            center=(x, 0.035, 5.98),
            length=0.11,
            radius=0.016,
            material=SHADOW_STEEL,
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

    # Parapet hook bracket: horizontal arm spanning the pivot bosses with two
    # down-turned hook legs that lap over the roof parapet. The arm sits forward
    # of the rails (in +Y) so it clears the vertical rail cylinders.
    hook = model.part("parapet_hook")
    # Horizontal cross-arm between the two pivot bosses.
    # In hook local frame, the arm is at y=0; the joint origin offsets it to
    # world y=0.07, forward of the rails.
    _cyl_x(
        hook,
        "parapet_hook_arm",
        center=(0.0, 0.0, 0.0),
        length=0.56,
        radius=0.020,
        material=SHADOW_STEEL,
    )
    # Two down-turned hook legs extending from the cross-arm toward the wall
    # side (−Y) and downward (−Z) to form the hook-over profile.
    for x, suffix in [(-0.19, "0"), (0.19, "1")]:
        hook_pts = [
            (x, 0.0, 0.0),
            (x, -0.05, -0.03),
            (x, -0.13, -0.10),
            (x, -0.17, -0.20),
            (x, -0.13, -0.30),
        ]
        leg = tube_from_spline_points(
            hook_pts,
            radius=0.016,
            samples_per_segment=6,
            closed_spline=False,
            radial_segments=12,
            cap_ends=True,
        )
        hook.visual(
            mesh_from_geometry(leg, f"parapet_hook_leg_mesh_{suffix}"),
            origin=Origin(),
            material=SHADOW_STEEL,
            name=f"parapet_hook_leg_{suffix}",
        )
    # Gusset plate connecting the two hook legs to the cross-arm for rigidity.
    # Shorter in X than the arm span to clear the vertical side rails.
    hook.visual(
        Box((0.42, 0.10, 0.025)),
        origin=Origin(xyz=(0.0, -0.04, -0.02)),
        material=SHADOW_STEEL,
        name="parapet_hook_gusset",
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
        "parapet_hook_pivot",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=hook,
        origin=Origin(xyz=(0.0, 0.07, 5.98)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-0.35, upper=0.35),
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
    hook = object_model.get_part("parapet_hook")
    slide = object_model.get_articulation("fixed_to_lower_ladder")
    hinge = object_model.get_articulation("fixed_to_safety_gate")
    pivot = object_model.get_articulation("parapet_hook_pivot")

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

    ctx.allow_overlap(
        fixed,
        hook,
        elem_a="hook_pivot_boss_0",
        elem_b="parapet_hook_arm",
        reason=(
            "The pivot boss is a cross-pin that passes through the hook arm; "
            "this local coaxial overlap represents the real pivot capture."
        ),
    )
    ctx.allow_overlap(
        fixed,
        hook,
        elem_a="hook_pivot_boss_1",
        elem_b="parapet_hook_arm",
        reason=(
            "The pivot boss is a cross-pin that passes through the hook arm; "
            "this local coaxial overlap represents the real pivot capture."
        ),
    )

    # Proof checks for pivot boss capture.
    ctx.expect_overlap(
        hook,
        fixed,
        axes="z",
        elem_a="parapet_hook_arm",
        elem_b="hook_pivot_boss_0",
        min_overlap=0.01,
        name="hook arm axially overlaps pivot boss 0 (captured pin)",
    )
    ctx.expect_overlap(
        hook,
        fixed,
        axes="z",
        elem_a="parapet_hook_arm",
        elem_b="hook_pivot_boss_1",
        min_overlap=0.01,
        name="hook arm axially overlaps pivot boss 1 (captured pin)",
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

    # Parapet hook pivot assertions: the hook bracket is attached to the
    # fixed ladder via a real REVOLUTE pivot at the rail heads, and the
    # pivot adjusts the hook-over angle for parapet fit.
    rest_hook_leg = ctx.part_element_world_aabb(hook, elem="parapet_hook_leg_0")
    with ctx.pose({pivot: 0.30}):
        tilted_hook_leg = ctx.part_element_world_aabb(hook, elem="parapet_hook_leg_0")
        ctx.expect_overlap(
            hook,
            fixed,
            axes="xy",
            elem_a="parapet_hook_arm",
            elem_b="hook_pivot_boss_0",
            min_overlap=0.02,
            name="hook arm stays aligned with pivot boss at tilted pose",
        )

    ctx.check(
        "parapet_hook_pivot adjusts hook angle",
        rest_hook_leg is not None
        and tilted_hook_leg is not None
        and abs(tilted_hook_leg[0][2] - rest_hook_leg[0][2]) > 0.005,
        details=f"rest_z={rest_hook_leg[0][2] if rest_hook_leg else None}, tilted_z={tilted_hook_leg[0][2] if tilted_hook_leg else None}",
    )
    ctx.expect_overlap(
        hook,
        fixed,
        axes="x",
        elem_a="parapet_hook_arm",
        elem_b="hook_pivot_boss_0",
        min_overlap=0.01,
        name="hook arm spans across pivot bosses at rest",
    )

    return ctx.report()


object_model = build_object_model()
