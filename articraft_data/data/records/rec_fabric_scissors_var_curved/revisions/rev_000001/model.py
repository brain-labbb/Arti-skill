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

# Gentle upward arc for trimming blades: tips rise ~12 mm above the pivot plane.
CURVE_RISE = 0.012
CURVE_REF_X = -0.222


def _arc_z(x: float) -> float:
    """Gentle upward parabolic arc: blade tips curve up off the handle plane."""
    if x >= 0.0:
        return 0.0
    t = min(1.0, abs(x) / abs(CURVE_REF_X))
    return CURVE_RISE * t * t


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


def _apply_curve(flat_geom: MeshGeometry, *, curve_fn=_arc_z) -> MeshGeometry:
    """Apply the upward trimming arc to a flat extrusion mesh.

    Takes a correctly-built flat mesh (from ExtrudeGeometry or
    ExtrudeWithHolesGeometry) and bends each vertex upward along a
    gentle parabolic arc based on its X distance from the pivot.
    """
    curved = MeshGeometry()
    for x, y, z in flat_geom.vertices:
        curved.add_vertex(x, y, z + curve_fn(x))
    for face in flat_geom.faces:
        curved.add_face(*face)
    return curved


def _curved_extrude_with_holes(
    outer: list[tuple[float, float]],
    holes: list[list[tuple[float, float]]],
    thickness: float,
) -> MeshGeometry:
    """Curved extrusion with through-holes for trimming blades."""
    flat = ExtrudeWithHolesGeometry(outer, holes, thickness, center=True)
    return _apply_curve(flat)


def _curved_extrude(
    profile: list[tuple[float, float]],
    thickness: float,
) -> MeshGeometry:
    """Simple curved extrusion (no holes) for blade bevels and spine facets."""
    flat = ExtrudeGeometry(profile, thickness, center=True)
    return _apply_curve(flat)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="fabric_tailor_shears",
        meta={
            "reference_note": (
                "Image and category agree: the core object is fabric scissors / tailor shears. "
                "Variant: curved trimming blades that let the tailor trim flush against a surface."
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

    # ── Parent half: upper blade with gentle upward curve ──────────────────
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
            _curved_extrude_with_holes(
                upper_plate_profile,
                [_circle_profile(0.0085, segments=40)],
                BLADE_THICKNESS,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=gunmetal,
        name="blade_plate",
    )
    upper_shear.visual(
        _mesh(
            "upper_edge_bevel",
            _curved_extrude(
                [
                    (-0.212, 0.026),
                    (-0.034, -0.003),
                    (-0.025, 0.006),
                    (-0.205, 0.037),
                ],
                0.0008,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00435)),
        material=polished_edge,
        name="edge_bevel",
    )
    upper_shear.visual(
        _mesh(
            "upper_spine_facet",
            _curved_extrude(
                [
                    (-0.204, 0.042),
                    (-0.060, 0.052),
                    (-0.050, 0.044),
                    (-0.190, 0.034),
                ],
                0.0006,
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

    # ── Child half: lower blade with matching upward curve ─────────────────
    lower_plate_profile = [
        (-0.220, -0.010),  # lower blade point just below the upper point
        (-0.037, -0.045),  # lower back edge
        (0.018, -0.030),
        (0.071, -0.048),  # tang into lower handle
        (0.083, -0.065),
        (0.039, -0.059),
        (0.024, 0.014),  # upper pivot shoulder
        (-0.033, 0.004),
        (-0.211, 0.006),  # cutting edge
    ]
    lower_shear.visual(
        _mesh(
            "lower_blade_plate",
            _curved_extrude_with_holes(
                lower_plate_profile,
                [_circle_profile(0.0095, segments=40)],
                BLADE_THICKNESS,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=gunmetal,
        name="blade_plate",
    )
    lower_shear.visual(
        _mesh(
            "lower_edge_bevel",
            _curved_extrude(
                [
                    (-0.211, 0.006),
                    (-0.032, 0.004),
                    (-0.034, -0.006),
                    (-0.205, -0.004),
                ],
                0.0008,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.00065)),
        material=polished_edge,
        name="edge_bevel",
    )
    lower_shear.visual(
        _mesh(
            "lower_spine_facet",
            _curved_extrude(
                [
                    (-0.203, -0.010),
                    (-0.045, -0.039),
                    (-0.052, -0.031),
                    (-0.192, -0.005),
                ],
                0.0006,
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

    # Curved blades: the Z projection ranges overlap due to the arc, so we
    # verify 3D clearance via contact distance rather than axis projection gap.
    ctx.expect_contact(
        upper,
        lower,
        elem_a="blade_plate",
        elem_b="blade_plate",
        contact_tol=0.004,
        name="curved blade_plate profiles maintain close shear clearance",
    )
    # The screw shank passes through the lower blade pivot hole. The tiny
    # overlap volume is a numerical artifact at the hole boundary.
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="blade_plate",
        elem_b="screw_shank",
        reason="Screw shank passes through the lower blade pivot hole; small overlap "
               "volume is the exact-collision artifact at the hole boundary.",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        elem_a="screw_shank",
        elem_b="blade_plate",
        min_overlap=0.008,
        name="screw_shank sits within lower blade pivot hole footprint",
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

    # Curved trimming blade verification: blade tips must curve upward
    # (higher in Z) relative to the pivot plane.
    upper_blade_aabb = ctx.part_element_world_aabb(upper, elem="blade_plate")
    lower_blade_aabb = ctx.part_element_world_aabb(lower, elem="blade_plate")
    ctx.check(
        "upper blade_plate tips curve upward off handle plane",
        upper_blade_aabb is not None
        and upper_blade_aabb[1][2] > 0.014,
        details=f"upper blade AABB max_z={upper_blade_aabb[1][2] if upper_blade_aabb else None}",
    )
    ctx.check(
        "lower blade_plate tips curve upward off handle plane",
        lower_blade_aabb is not None
        and lower_blade_aabb[1][2] > 0.008,
        details=f"lower blade AABB max_z={lower_blade_aabb[1][2] if lower_blade_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
