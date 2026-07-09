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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="fabric_tailor_shears",
        meta={
            "reference_note": (
                "Image and category agree: the core object is fabric scissors / tailor shears."
            )
        },
    )

    gunmetal = model.material("blued_steel", rgba=(0.12, 0.14, 0.18, 1.0))
    polished_edge = model.material("polished_bevel", rgba=(0.72, 0.77, 0.82, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.016, 0.019, 1.0))
    glossy_highlight = model.material("soft_highlight", rgba=(0.10, 0.11, 0.13, 1.0))
    brass = model.material("brass_screw", rgba=(0.96, 0.73, 0.34, 1.0))
    dark_slot = model.material("screw_slot_dark", rgba=(0.03, 0.025, 0.018, 1.0))

    upper_shear = model.part("upper_shear")
    lower_shear = model.part("lower_shear")

    # Parent half: the long upper blade, forged tang, thumb-loop handle, and visible screw head.
    upper_plate_profile = [
        (-0.222, 0.043),  # sharp point
        (-0.058, 0.055),  # thick back of the blade
        (-0.018, 0.038),
        (0.018, 0.018),  # pivot shoulder
        (0.073, 0.018),  # metal tang entering the handle
        (0.080, 0.002),
        (0.030, -0.017),
        (-0.032, -0.004),
        (-0.213, 0.026),  # honed cutting side back to the tip
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

    upper_shear.visual(
        _mesh(
            "upper_handle_loop",
            _ring_extrude_geometry(
                _oval_profile(0.112, 0.078, center=(0.135, 0.024), angle=-0.18),
                _oval_profile(0.080, 0.048, center=(0.135, 0.024), angle=-0.18),
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=black_plastic,
        name="handle_loop",
    )
    upper_shear.visual(
        _mesh(
            "upper_handle_neck",
            ExtrudeGeometry(
                [
                    (0.049, 0.001),
                    (0.094, -0.006),
                    (0.103, 0.006),
                    (0.064, 0.026),
                ],
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=black_plastic,
        name="handle_neck",
    )
    upper_shear.visual(
        _mesh(
            "upper_handle_lip",
            ExtrudeGeometry(
                _oval_profile(0.030, 0.018, center=(0.186, -0.004), angle=-0.28, segments=40),
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=black_plastic,
        name="handle_lip",
    )
    upper_shear.visual(
        _mesh(
            "upper_grip_highlight",
            ExtrudeGeometry(
                [
                    (0.092, 0.047),
                    (0.160, 0.052),
                    (0.165, 0.059),
                    (0.096, 0.054),
                ],
                0.0006,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00875)),
        material=glossy_highlight,
        name="grip_highlight",
    )

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

    # Child half: broad duckbill / pelican paddle blade lies below the upper pointed blade.
    # The flat top face of the paddle rides against the upper cutting edge so the
    # paddle lifts and protects the under-layer fabric while the upper blade trims.
    lower_plate_profile = [
        # Rounded blunt paddle tip (no sharp point)
        (-0.226, -0.016),  # bottom of rounded tip
        (-0.234, -0.003),  # leftmost rounded nose
        (-0.226, 0.012),   # top of rounded tip
        # Flat top surface — the upper blade cutting edge rides on this face
        (-0.190, 0.017),
        (-0.148, 0.019),
        (-0.100, 0.018),
        (-0.052, 0.015),
        (0.024, 0.014),    # upper pivot shoulder
        # Tang into lower handle (preserved from parent)
        (0.039, -0.059),
        (0.083, -0.065),
        (0.071, -0.048),
        (0.018, -0.030),
        # Broad bottom of paddle — wide flat underside
        (-0.052, -0.034),
        (-0.100, -0.037),
        (-0.148, -0.037),
        (-0.190, -0.032),
        (-0.218, -0.025),
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
    # Polished strip along the flat top of the paddle (riding surface)
    lower_shear.visual(
        _mesh(
            "lower_edge_bevel",
            ExtrudeGeometry(
                [
                    (-0.222, 0.012),
                    (-0.052, 0.015),
                    (-0.054, 0.008),
                    (-0.218, 0.005),
                ],
                0.0008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00065)),
        material=polished_edge,
        name="edge_bevel",
    )
    # Polished facet along the broad bottom of the paddle
    lower_shear.visual(
        _mesh(
            "lower_spine_facet",
            ExtrudeGeometry(
                [
                    (-0.210, -0.022),
                    (-0.055, -0.034),
                    (-0.060, -0.028),
                    (-0.200, -0.017),
                ],
                0.0006,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00075)),
        material=polished_edge,
        name="spine_facet",
    )

    lower_shear.visual(
        _mesh(
            "lower_handle_loop",
            _ring_extrude_geometry(
                _oval_profile(0.154, 0.080, center=(0.132, -0.094), angle=-0.57),
                _oval_profile(0.112, 0.046, center=(0.132, -0.094), angle=-0.57),
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=black_plastic,
        name="handle_loop",
    )
    lower_shear.visual(
        _mesh(
            "lower_handle_neck",
            ExtrudeGeometry(
                [
                    (0.034, -0.038),
                    (0.091, -0.052),
                    (0.081, -0.069),
                    (0.027, -0.057),
                ],
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=black_plastic,
        name="handle_neck",
    )
    lower_shear.visual(
        _mesh(
            "lower_pinky_lip",
            ExtrudeGeometry(
                _oval_profile(0.034, 0.022, center=(0.199, -0.138), angle=-0.57, segments=44),
                HANDLE_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0050)),
        material=black_plastic,
        name="pinky_lip",
    )
    lower_shear.visual(
        _mesh(
            "lower_grip_highlight",
            ExtrudeGeometry(
                [
                    (0.086, -0.119),
                    (0.166, -0.148),
                    (0.171, -0.140),
                    (0.092, -0.111),
                ],
                0.0006,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00125)),
        material=glossy_highlight,
        name="grip_highlight",
    )

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

    # Duckbill paddle assertion: the lower blade_plate must be a broad rounded
    # paddle (not a pointed blade), with y-extent significantly wider than a
    # pointed tip blade would produce.
    lower_plate_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
    upper_plate_aabb = ctx.part_element_world_aabb(upper, elem="blade_plate")
    if lower_plate_aabb is not None and upper_plate_aabb is not None:
        lower_y = lower_plate_aabb[1][1] - lower_plate_aabb[0][1]
        upper_y = upper_plate_aabb[1][1] - upper_plate_aabb[0][1]
        ctx.check(
            "lower blade_plate is a broad duckbill paddle",
            lower_y > 0.044,
            details=f"lower paddle y-extent={lower_y:.4f}m, expected >0.044m",
        )
        ctx.check(
            "lower paddle is wider than upper pointed blade",
            lower_y > upper_y,
            details=f"lower_y={lower_y:.4f}, upper_y={upper_y:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
