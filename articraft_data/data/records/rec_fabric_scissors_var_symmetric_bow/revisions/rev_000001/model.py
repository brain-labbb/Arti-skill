from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


BLADE_THICKNESS = 0.0030
HANDLE_THICKNESS = 0.0070


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    ca = math.cos(angle)
    sa = math.sin(angle)
    return [(ca * x - sa * y + dx, sa * x + ca * y + dy) for x, y in profile]


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def _oval_profile(
    width: float,
    height: float,
    *,
    center: tuple[float, float],
    angle: float = 0.0,
    exponent: float = 2.45,
    segments: int = 64,
) -> list[tuple[float, float]]:
    return _transform_profile(
        superellipse_profile(width, height, exponent=exponent, segments=segments),
        dx=center[0],
        dy=center[1],
        angle=angle,
    )


def _ring_extrude_geometry(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    height: float,
    *,
    center: bool = True,
) -> MeshGeometry:
    if len(outer_profile) != len(inner_profile):
        raise ValueError("Ring profiles must have matching segment counts")
    if len(outer_profile) < 3:
        raise ValueError("Ring profiles need at least three points")

    z0 = -height / 2.0 if center else 0.0
    z1 = z0 + height
    geom = MeshGeometry()

    outer_bottom = [geom.add_vertex(x, y, z0) for x, y in outer_profile]
    outer_top = [geom.add_vertex(x, y, z1) for x, y in outer_profile]
    inner_bottom = [geom.add_vertex(x, y, z0) for x, y in inner_profile]
    inner_top = [geom.add_vertex(x, y, z1) for x, y in inner_profile]

    count = len(outer_profile)
    for i in range(count):
        j = (i + 1) % count

        # Outer and inner side walls.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])

        # Top and bottom annular caps, segment by segment, so the center stays open.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return geom


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


# Shared symmetric round-bow geometry: both shears use identical finger loops
# centered on the blade cutting axis (y = 0).
BOW_CENTER = (0.130, 0.0)
BOW_OUTER_DIA = 0.076
BOW_INNER_DIA = 0.050
BOW_SEGMENTS = 56


def _symmetric_bow():
    """Return (outer_profile, inner_profile) for a round finger bow on the blade axis."""
    outer = _oval_profile(
        BOW_OUTER_DIA,
        BOW_OUTER_DIA,
        center=BOW_CENTER,
        angle=0.0,
        exponent=2.0,
        segments=BOW_SEGMENTS,
    )
    inner = _oval_profile(
        BOW_INNER_DIA,
        BOW_INNER_DIA,
        center=BOW_CENTER,
        angle=0.0,
        exponent=2.0,
        segments=BOW_SEGMENTS,
    )
    return outer, inner


# Straight tang/neck profile: a bar from the blade tang end into the bow ring,
# centered on y = 0 so the bow is collinear with the cutting axis.
_NECK_PROFILE = [
    (0.075, -0.006),
    (0.075, 0.006),
    (0.100, 0.006),
    (0.100, -0.006),
]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="fabric_scissors_inline",
        meta={
            "reference_note": (
                "Inline symmetric variant of fabric scissors: two equal round finger bows "
                "centered on the blade cutting axis, no bent-handle offset or pinky rest."
            )
        },
    )

    gunmetal = model.material("blued_steel", rgba=(0.12, 0.14, 0.18, 1.0))
    polished_edge = model.material("polished_bevel", rgba=(0.72, 0.77, 0.82, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.016, 0.019, 1.0))
    brass = model.material("brass_screw", rgba=(0.96, 0.73, 0.34, 1.0))
    dark_slot = model.material("screw_slot_dark", rgba=(0.03, 0.025, 0.018, 1.0))

    upper_shear = model.part("upper_shear")
    lower_shear = model.part("lower_shear")

    # Shared bow profiles — identical for both shears (symmetric inline).
    bow_outer, bow_inner = _symmetric_bow()

    # ── Upper shear: blade + straight tang + round bow + screw head ──────────

    # Blade plate: cutting geometry preserved from parent; tang straightened
    # to stay on the blade centerline (y ≈ 0) instead of cranking upward.
    upper_plate_profile = [
        (-0.222, 0.043),   # sharp point
        (-0.058, 0.055),   # thick back of the blade
        (-0.018, 0.038),
        (0.018, 0.018),    # pivot shoulder
        (0.068, 0.010),    # straight tang tapering toward centerline
        (0.078, 0.000),    # tang end at blade axis
        (0.030, -0.014),   # return below centerline for pivot crossing
        (-0.032, -0.004),
        (-0.213, 0.026),   # honed cutting side back to the tip
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

    # Symmetric round finger bow (identical geometry for both shears).
    upper_shear.visual(
        _mesh(
            "upper_handle_loop",
            _ring_extrude_geometry(bow_outer, bow_inner, HANDLE_THICKNESS, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=black_plastic,
        name="handle_loop",
    )
    # Straight tang/neck: centered on y = 0, no bent-handle crank.
    upper_shear.visual(
        _mesh(
            "upper_handle_neck",
            ExtrudeGeometry(_NECK_PROFILE, HANDLE_THICKNESS, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=black_plastic,
        name="handle_neck",
    )

    # Screw hardware (on upper shear face).
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

    # ── Lower shear: blade + straight tang + round bow ───────────────────────

    # Blade plate: cutting geometry preserved; tang straightened to y ≈ 0
    # instead of cranking downward for the offset finger loop.
    lower_plate_profile = [
        (-0.220, -0.010),  # lower blade point just below the upper point
        (-0.037, -0.045),  # lower back edge
        (0.018, -0.018),   # pivot shoulder (inline tang)
        (0.068, -0.010),   # straight tang tapering toward centerline
        (0.078, 0.000),    # tang end at blade axis
        (0.030, 0.008),    # narrow return
        (0.024, 0.014),    # upper pivot shoulder
        (-0.033, 0.004),
        (-0.211, 0.006),   # cutting edge
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

    # Same symmetric round bow as upper shear — equal size, centered on axis.
    lower_shear.visual(
        _mesh(
            "lower_handle_loop",
            _ring_extrude_geometry(bow_outer, bow_inner, HANDLE_THICKNESS, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=black_plastic,
        name="handle_loop",
    )
    # Straight tang/neck: same profile as upper, no offset crank.
    lower_shear.visual(
        _mesh(
            "lower_handle_neck",
            ExtrudeGeometry(_NECK_PROFILE, HANDLE_THICKNESS, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=black_plastic,
        name="handle_neck",
    )

    # ── Pivot articulation ────────────────────────────────────────────────────

    model.articulation(
        "pivot_screw",
        ArticulationType.REVOLUTE,
        parent=upper_shear,
        child=lower_shear,
        # The child frame is at the screw axis. Positive rotation swings the
        # lower blade farther below the upper blade, opening the shears.
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.28, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_shear")
    lower = object_model.get_part("lower_shear")
    pivot = object_model.get_articulation("pivot_screw")

    # ── Preserved parent checks: blade stacking, pivot capture, opening ──────

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

    rest_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
    with ctx.pose({pivot: 0.55}):
        open_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a="screw_head",
            elem_b="blade_plate",
            min_overlap=0.008,
            name="pivot remains captured when shears open",
        )

    ctx.check(
        "positive pivot opens the lower blade",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.040,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    # ── Variant-specific checks: symmetric inline handle bows ────────────────

    # Both handle_loops are stacked at the same XY footprint (inline, not offset).
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="handle_loop",
        elem_b="handle_loop",
        min_overlap=0.030,
        name="handle_loop on upper_shear and lower_shear share the same XY footprint (symmetric inline)",
    )

    # Both handle_loops overlap substantially on Y, proving neither is offset
    # above or below the blade centerline — they are both centered at y ≈ 0.
    ctx.expect_overlap(
        upper,
        lower,
        axes="y",
        elem_a="handle_loop",
        elem_b="handle_loop",
        min_overlap=0.040,
        name="handle_loop bows are centered on the blade axis (no asymmetric Y offset)",
    )

    # Handle necks connect blade tang to bow (inline straight tang).
    ctx.expect_overlap(
        upper,
        upper,
        axes="x",
        elem_a="handle_neck",
        elem_b="blade_plate",
        min_overlap=0.002,
        name="upper handle_neck connects blade tang to inline bow",
    )
    ctx.expect_overlap(
        lower,
        lower,
        axes="x",
        elem_a="handle_neck",
        elem_b="blade_plate",
        min_overlap=0.002,
        name="lower handle_neck connects blade tang to inline bow",
    )

    return ctx.report()


object_model = build_object_model()
