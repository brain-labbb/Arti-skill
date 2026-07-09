from __future__ import annotations

import math

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
    mesh_from_geometry,
    superellipse_profile,
)


BLADE_THICKNESS = 0.0030
YOKE_WIDTH = 0.012  # spring strip width (Z extrusion)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 48):
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="fabric_thread_snips",
        meta={
            "reference_note": (
                "Variant: spring-action sewing snips with leaf-spring yoke bridge. "
                "Category Textiles_Fabric/Fabric scissors."
            ),
            "variant_note": (
                "Skeleton changed from two-arm finger-loop shears to single "
                "sprung squeeze yoke (thread-snip / nigiri skeleton)."
            ),
        },
    )

    gunmetal = model.material("blued_steel", rgba=(0.12, 0.14, 0.18, 1.0))
    polished_edge = model.material("polished_bevel", rgba=(0.72, 0.77, 0.82, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.48, 0.52, 0.58, 1.0))
    brass = model.material("brass_screw", rgba=(0.96, 0.73, 0.34, 1.0))
    dark_slot = model.material("screw_slot_dark", rgba=(0.03, 0.025, 0.018, 1.0))

    upper_shear = model.part("upper_shear")
    lower_shear = model.part("lower_shear")

    # ------------------------------------------------------------------ #
    # Upper shear: blade, bevel, spine, screw hardware (preserved)
    # ------------------------------------------------------------------ #
    upper_plate_profile = [
        (-0.222, 0.043),
        (-0.058, 0.055),
        (-0.018, 0.038),
        (0.018, 0.018),
        (0.073, 0.018),
        (0.080, 0.002),
        (0.030, -0.017),
        (-0.032, -0.004),
        (-0.213, 0.026),
    ]
    upper_shear.visual(
        _mesh(
            "upper_blade_plate",
            ExtrudeWithHolesGeometry(
                upper_plate_profile,
                [_circle_profile(0.0085, segments=40)],
                BLADE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=gunmetal,
        name="blade_plate",
    )
    upper_shear.visual(
        _mesh(
            "upper_edge_bevel",
            ExtrudeGeometry(
                [
                    (-0.212, 0.026),
                    (-0.034, -0.003),
                    (-0.025, 0.006),
                    (-0.205, 0.037),
                ],
                0.0008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00435)),
        material=polished_edge,
        name="edge_bevel",
    )
    upper_shear.visual(
        _mesh(
            "upper_spine_facet",
            ExtrudeGeometry(
                [
                    (-0.204, 0.042),
                    (-0.060, 0.052),
                    (-0.050, 0.044),
                    (-0.190, 0.034),
                ],
                0.0006,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00425)),
        material=polished_edge,
        name="spine_facet",
    )

    # Screw hardware (preserved)
    upper_shear.visual(
        Cylinder(radius=0.0168, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=brass,
        name="screw_head",
    )
    upper_shear.visual(
        Cylinder(radius=0.0100, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
        material=brass,
        name="screw_boss",
    )
    upper_shear.visual(
        Cylinder(radius=0.0050, length=0.0125),
        origin=Origin(xyz=(0.0, 0.0, -0.0018)),
        material=brass,
        name="screw_shank",
    )
    upper_shear.visual(
        Box((0.024, 0.0032, 0.0009)),
        origin=Origin(xyz=(0.0, 0.0, 0.01135), rpy=(0.0, 0.0, -0.38)),
        material=dark_slot,
        name="screw_slot",
    )

    # ------------------------------------------------------------------ #
    # Upper yoke leg (NEW): upper half of the U-shaped leaf spring
    # Starts at the upper tang, curves rearward and down to the bend apex
    # ------------------------------------------------------------------ #
    upper_yoke_profile = [
        # Start extends into blade tang area for connectivity (blade y_max≈0.018)
        (0.070, 0.0205),
        (0.085, 0.0240),
        (0.102, 0.0255),
        (0.116, 0.0240),
        (0.128, 0.0195),
        (0.139, 0.0125),
        (0.147, 0.0040),
        (0.153, -0.0065),
        (0.156, -0.0180),   # outer meeting point (extends past bend for overlap)
        (0.153, -0.0180),   # inner meeting point
        (0.150, -0.0065),
        (0.144, 0.0030),
        (0.136, 0.0100),
        (0.126, 0.0165),
        (0.113, 0.0210),
        (0.099, 0.0225),
        (0.085, 0.0210),
        (0.070, 0.0165),   # inner start overlaps with blade (blade y_max≈0.018)
    ]
    upper_shear.visual(
        _mesh(
            "upper_spring_yoke",
            ExtrudeGeometry(upper_yoke_profile, YOKE_WIDTH, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=spring_steel,
        name="spring_yoke",
    )

    # Small rivet where upper yoke attaches to tang (bridges yoke and blade)
    upper_shear.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.076, 0.018, 0.0)),
        material=brass,
        name="yoke_rivet",
    )

    # ------------------------------------------------------------------ #
    # Lower shear: blade, bevel, spine (preserved)
    # ------------------------------------------------------------------ #
    lower_plate_profile = [
        (-0.220, -0.010),
        (-0.037, -0.045),
        (0.018, -0.030),
        (0.071, -0.048),
        (0.083, -0.065),
        (0.039, -0.059),
        (0.024, 0.014),
        (-0.033, 0.004),
        (-0.211, 0.006),
    ]
    lower_shear.visual(
        _mesh(
            "lower_blade_plate",
            ExtrudeWithHolesGeometry(
                lower_plate_profile,
                [_circle_profile(0.0095, segments=40)],
                BLADE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=gunmetal,
        name="blade_plate",
    )
    lower_shear.visual(
        _mesh(
            "lower_edge_bevel",
            ExtrudeGeometry(
                [
                    (-0.211, 0.006),
                    (-0.032, 0.004),
                    (-0.034, -0.006),
                    (-0.205, -0.004),
                ],
                0.0008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00065)),
        material=polished_edge,
        name="edge_bevel",
    )
    lower_shear.visual(
        _mesh(
            "lower_spine_facet",
            ExtrudeGeometry(
                [
                    (-0.203, -0.010),
                    (-0.045, -0.039),
                    (-0.052, -0.031),
                    (-0.192, -0.005),
                ],
                0.0006,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00075)),
        material=polished_edge,
        name="spine_facet",
    )

    # ------------------------------------------------------------------ #
    # Lower yoke leg (NEW): lower half of the U-shaped leaf spring
    # Starts at the lower tang, curves rearward and up to the bend apex
    # At q=0 (closed), meets the upper yoke at the shared bend point
    # ------------------------------------------------------------------ #
    lower_yoke_profile = [
        # Start extends into blade tang area for connectivity
        (0.072, -0.0590),   # outer start overlaps with blade tang
        (0.088, -0.0560),
        (0.105, -0.0505),
        (0.120, -0.0430),
        (0.133, -0.0335),
        (0.144, -0.0235),
        (0.156, -0.0120),   # outer meeting point (extends past bend for overlap)
        (0.153, -0.0120),   # inner meeting point
        (0.141, -0.0210),
        (0.130, -0.0300),
        (0.117, -0.0385),
        (0.103, -0.0455),
        (0.088, -0.0510),
        (0.072, -0.0545),   # inner start
    ]
    lower_shear.visual(
        _mesh(
            "lower_spring_yoke",
            ExtrudeGeometry(lower_yoke_profile, YOKE_WIDTH, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=spring_steel,
        name="spring_yoke",
    )

    # Small rivet where lower yoke attaches to tang (bridges yoke and blade)
    lower_shear.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.078, -0.056, 0.0)),
        material=brass,
        name="yoke_rivet",
    )

    # ------------------------------------------------------------------ #
    # Articulation: pivot_screw revolute
    # Spring-biased rest at the open limit (upper bound).
    # q=0: closed (blades together), q=upper: open (spring rest).
    # Positive q rotates lower blade away from upper blade (opening).
    # ------------------------------------------------------------------ #
    model.articulation(
        "pivot_screw",
        ArticulationType.REVOLUTE,
        parent=upper_shear,
        child=lower_shear,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_shear")
    lower = object_model.get_part("lower_shear")
    pivot = object_model.get_articulation("pivot_screw")

    # --- Preserved blade/pivot checks ---
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="blade_plate",
        negative_elem="blade_plate",
        min_gap=0.001,
        max_gap=0.004,
        name="stacked blades have metal clearance",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="blade_plate",
        elem_b="blade_plate",
        min_overlap=0.025,
        name="blade plates visibly cross at pivot",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="screw_head",
        elem_b="blade_plate",
        min_overlap=0.008,
        name="brass screw is centered over lower pivot hole",
    )

    # --- Variant-specific: spring yoke checks ---
    # The two yoke halves contact/overlap at the bend apex when closed,
    # forming the continuous U-shaped leaf spring.
    ctx.expect_gap(
        upper,
        lower,
        axis="y",
        positive_elem="spring_yoke",
        negative_elem="spring_yoke",
        max_penetration=0.010,
        max_gap=0.002,
        name="spring_yoke halves contact at bend when closed",
    )

    # Allow the intentional local overlap at the yoke bend contact zone
    ctx.allow_overlap(
        upper,
        lower,
        elem_a="spring_yoke",
        elem_b="spring_yoke",
        reason=(
            "The two yoke halves intentionally overlap at the bend apex to "
            "represent the continuous U-shaped leaf spring contact."
        ),
    )

    # Yoke rivets are on their respective arms
    ctx.expect_overlap(
        upper,
        upper,
        axes="xy",
        elem_a="yoke_rivet",
        elem_b="spring_yoke",
        min_overlap=0.003,
        name="upper yoke rivet is on the spring strip",
    )
    ctx.expect_overlap(
        lower,
        lower,
        axes="xy",
        elem_a="yoke_rivet",
        elem_b="spring_yoke",
        min_overlap=0.003,
        name="lower yoke rivet is on the spring strip",
    )

    # --- Articulation: spring-biased open ---
    # Pivot screw lower limit = 0 (closed), upper > 0 (open/rest)
    ctx.check(
        "pivot_screw spring-biased rest at open limit",
        pivot.motion_limits.lower == 0.0 and pivot.motion_limits.upper > 0.0,
        details=(
            f"lower={pivot.motion_limits.lower}, "
            f"upper={pivot.motion_limits.upper}"
        ),
    )

    # Rest pose (q=0, closed) vs open pose — prove the mechanism opens
    rest_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
    with ctx.pose({pivot: 0.42}):
        open_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a="screw_head",
            elem_b="blade_plate",
            min_overlap=0.008,
            name="pivot remains captured when snips open",
        )

    ctx.check(
        "positive pivot opens the lower blade (squeeze-to-close)",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.030,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    # At open pose, yoke halves separate (spring spread)
    closed_yoke_overlap = True  # will verify below
    with ctx.pose({pivot: 0.42}):
        # At open pose the yoke halves should have moved apart
        open_yoke_aabb_upper = ctx.part_element_world_aabb(upper, elem="spring_yoke")
        open_yoke_aabb_lower = ctx.part_element_world_aabb(lower, elem="spring_yoke")

    closed_yoke_aabb_upper = ctx.part_element_world_aabb(upper, elem="spring_yoke")
    closed_yoke_aabb_lower = ctx.part_element_world_aabb(lower, elem="spring_yoke")

    # At closed, yoke bend ends are close; at open, they separate in Y
    if (
        closed_yoke_aabb_lower is not None
        and open_yoke_aabb_lower is not None
    ):
        closed_yoke_lower_max_y = closed_yoke_aabb_lower[1][1]
        open_yoke_lower_max_y = open_yoke_aabb_lower[1][1]
        ctx.check(
            "spring yoke spreads when snips open",
            open_yoke_lower_max_y > closed_yoke_lower_max_y + 0.010,
            details=(
                f"closed_lower_max_y={closed_yoke_lower_max_y}, "
                f"open_lower_max_y={open_yoke_lower_max_y}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
