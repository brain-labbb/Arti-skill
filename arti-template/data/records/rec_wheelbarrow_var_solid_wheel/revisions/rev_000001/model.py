from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_PIVOT = (0.0, -0.60, 0.25)


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _barrow_origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(
        xyz=(x - AXLE_PIVOT[0], y - AXLE_PIVOT[1], z - AXLE_PIVOT[2]),
        rpy=rpy,
    )


def _barrow_point(point):
    return tuple(point[i] - AXLE_PIVOT[i] for i in range(3))


def _barrow_mesh(geometry: MeshGeometry) -> MeshGeometry:
    return geometry.translate(-AXLE_PIVOT[0], -AXLE_PIVOT[1], -AXLE_PIVOT[2])


def _rounded_rect_loop(width: float, length: float, z: float, segments: int = 64) -> list[tuple[float, float, float]]:
    """Superellipse-like loop in the XY plane, used for the tapered metal tray."""
    pts: list[tuple[float, float, float]] = []
    exponent = 3.6
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        ct = math.cos(t)
        st = math.sin(t)
        x = (width * 0.5) * math.copysign(abs(ct) ** (2.0 / exponent), ct)
        y = (length * 0.5) * math.copysign(abs(st) ** (2.0 / exponent), st)
        pts.append((x, y, z))
    return pts


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _wheelbarrow_tray_geometry() -> MeshGeometry:
    """A hollow, open, deep galvanized tray with a rolled/folded upper lip."""
    geom = MeshGeometry()
    n = 72

    # Four corresponding loops: tapered underside, inner floor, inner mouth, and a
    # larger rolled rim.  The open top stays visibly hollow instead of capped.
    outer_bottom = _rounded_rect_loop(0.42, 0.58, 0.32, n)
    inner_bottom = _rounded_rect_loop(0.32, 0.48, 0.36, n)
    inner_top = _rounded_rect_loop(0.72, 1.04, 0.67, n)
    outer_lip = _rounded_rect_loop(0.86, 1.18, 0.70, n)

    loops = []
    for loop in (outer_bottom, inner_bottom, inner_top, outer_lip):
        idxs = [geom.add_vertex(*p) for p in loop]
        loops.append(idxs)

    ob, ib, it, ol = loops
    center_under = geom.add_vertex(0.0, 0.0, 0.32)
    center_floor = geom.add_vertex(0.0, 0.0, 0.36)

    for i in range(n):
        j = (i + 1) % n
        # Outer sloped steel wall
        _add_quad(geom, ob[i], ob[j], ol[j], ol[i])
        # Inner sloped wall of the usable basin
        _add_quad(geom, it[i], it[j], ib[j], ib[i])
        # Folded/rolled top lip bridges outer shell to inner opening.
        _add_quad(geom, ol[i], ol[j], it[j], it[i])
        # Bottom thickness bridge between outside underside and inside floor.
        _add_quad(geom, ib[i], ib[j], ob[j], ob[i])
        # Underside and interior floor caps.
        geom.add_face(center_under, ob[j], ob[i])
        geom.add_face(center_floor, ib[i], ib[j])

    return geom


def _triangular_plate_geometry(
    x_center: float,
    thickness: float,
    yz_points: list[tuple[float, float]],
) -> MeshGeometry:
    """Small fork/bracket plate, a prism whose broad face lies in the YZ plane."""
    geom = MeshGeometry()
    x0 = x_center - thickness * 0.5
    x1 = x_center + thickness * 0.5
    front = [geom.add_vertex(x0, y, z) for y, z in yz_points]
    back = [geom.add_vertex(x1, y, z) for y, z in yz_points]
    n = len(yz_points)

    # Broad faces.
    for i in range(1, n - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])
    # Edge band.
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, front[i], front[j], back[j], back[i])
    return geom


def _solid_disc_wheel_geometry() -> MeshGeometry:
    """Solid flat-free molded disc wheel body with rounded tread edges.

    The profile revolves around the Z axis (LatheGeometry default),
    then rotates so the revolution axis aligns with X for the wheel spin axis.
    """
    profile = [
        (0.000, -0.0425),   # center axis, inner face
        (0.215, -0.0425),   # disc body, inner face
        (0.228, -0.038),    # shoulder rounding start
        (0.235, -0.028),    # tread rounding
        (0.238,  0.000),    # tread crown (slight barrel)
        (0.235,  0.028),    # tread rounding
        (0.228,  0.038),    # shoulder rounding start
        (0.215,  0.0425),   # disc body, outer face
        (0.000,  0.0425),   # center axis, outer face
    ]
    geom = LatheGeometry(profile, segments=48)
    geom.rotate_y(math.pi / 2.0)
    return geom


def _tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _barrow_tube_mesh(points: list[tuple[float, float, float]], radius: float, name: str):
    return _tube_mesh([_barrow_point(point) for point in points], radius, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_single_wheelbarrow",
        meta={
            "category": "Agricultural",
            "small_class": "Single-Wheelbarrow",
            "reference": "picture/Agricultural/Single-Wheelbarrow/001.png",
        },
    )

    galvanized = model.material("weathered_galvanized_tray", rgba=(0.58, 0.62, 0.62, 1.0))
    darker_galv = model.material("darker_scuffed_metal", rgba=(0.42, 0.45, 0.45, 1.0))
    green = model.material("dark_green_painted_steel", rgba=(0.02, 0.18, 0.11, 1.0))
    green_edge = model.material("worn_green_edges", rgba=(0.04, 0.25, 0.16, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    grip_rubber = model.material("grey_textured_grip", rgba=(0.20, 0.20, 0.22, 1.0))
    hardware = model.material("dark_bolt_hardware", rgba=(0.03, 0.03, 0.03, 1.0))

    axle_pivot = model.part("wheel_axle_pivot")
    barrow = model.part("barrow")

    # Deep tapered tray: approximately 0.86 m wide by 1.18 m long at the lip.
    barrow.visual(
        mesh_from_geometry(_barrow_mesh(_wheelbarrow_tray_geometry()), "deep_tray_shell"),
        material=galvanized,
        name="tray_shell",
    )

    # Slightly darker outside reinforcing straps and side panel seam ribs.
    for x, name in [(-0.360, "side_rib_0"), (0.360, "side_rib_1")]:
        barrow.visual(
            Box((0.012, 0.80, 0.028)),
            origin=_barrow_origin(x, 0.05, 0.555),
            material=darker_galv,
            name=name,
        )
    barrow.visual(
        Box((0.58, 0.030, 0.026)),
        origin=_barrow_origin(0.0, -0.535, 0.635),
        material=darker_galv,
        name="front_seam",
    )
    barrow.visual(
        Box((0.52, 0.035, 0.024)),
        origin=_barrow_origin(0.0, 0.535, 0.635),
        material=darker_galv,
        name="rear_seam",
    )

    # Continuous green tube frame and handles.
    left_rail_points = [
        (-0.33, -0.86, 0.22),
        (-0.33, -0.60, 0.25),
        (-0.32, -0.20, 0.30),
        (-0.31, 0.28, 0.35),
        (-0.31, 0.58, 0.56),
        (-0.31, 1.08, 0.88),
        (-0.31, 1.42, 0.95),
    ]
    right_rail_points = [(abs(x), y, z) for x, y, z in left_rail_points]
    barrow.visual(_barrow_tube_mesh(left_rail_points, 0.018, "left_handle_rail"), material=green, name="handle_rail_0")
    barrow.visual(_barrow_tube_mesh(right_rail_points, 0.018, "right_handle_rail"), material=green, name="handle_rail_1")

    front_loop_points = [
        (-0.33, -0.86, 0.22),
        (-0.18, -0.92, 0.205),
        (0.0, -0.94, 0.205),
        (0.18, -0.92, 0.205),
        (0.33, -0.86, 0.22),
    ]
    barrow.visual(_barrow_tube_mesh(front_loop_points, 0.018, "front_guard_loop"), material=green, name="front_guard")

    for y, z, nm in [(-0.22, 0.300, "front_cross_tube"), (0.28, 0.355, "rear_cross_tube")]:
        barrow.visual(
            Cylinder(radius=0.016, length=0.68),
            origin=_barrow_origin(0.0, y, z, (0.0, math.pi / 2.0, 0.0)),
            material=green,
            name=nm,
        )

    for x, nm in [(-0.31, "support_leg_0"), (0.31, "support_leg_1")]:
        leg_points = [
            (x, 0.06, 0.33),
            (x, 0.22, 0.18),
            (x, 0.40, 0.045),
            (x, 0.60, 0.045),
            (x, 0.72, 0.36),
        ]
        barrow.visual(_barrow_tube_mesh(leg_points, 0.016, nm), material=green_edge, name=nm)
        # Worn flat foot pad where the leg meets soil/concrete.
        barrow.visual(
            Box((0.070, 0.055, 0.012)),
            origin=_barrow_origin(x, 0.50, 0.035),
            material=green_edge,
            name=f"foot_pad_{0 if x < 0 else 1}",
        )

    # Tray mounting pads tie the tray underside to the frame instead of leaving
    # the shell visually floating above the tubes.
    for x in (-0.18, 0.18):
        for y in (-0.12, 0.24):
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            barrow.visual(
                Box((0.085, 0.060, 0.055)),
                origin=_barrow_origin(x, y, 0.297),
                material=green,
                name=f"tray_mount_{suffix}",
            )

    # Rubber handle grips with ring texture, mounted on the rear ends of the rails.
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        barrow.visual(
            Cylinder(radius=0.035, length=0.18),
            origin=_barrow_origin(x, 1.505, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name=f"grip_{idx}",
        )
        for ring_i, y in enumerate((1.445, 1.485, 1.525, 1.565)):
            barrow.visual(
                Cylinder(radius=0.037, length=0.008),
                origin=_barrow_origin(x, y, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
                material=hardware,
                name=f"grip_ring_{idx}_{ring_i}",
            )

    # Axle assembly and fork plates carry the single front wheel.
    barrow.visual(
        Cylinder(radius=0.022, length=0.74),
        origin=_barrow_origin(0.0, -0.60, 0.25, (0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="axle",
    )
    plate_yz = [(-0.735, 0.185), (-0.505, 0.205), (-0.475, 0.318), (-0.630, 0.355), (-0.750, 0.292)]
    for x, idx in [(-0.090, 0), (0.090, 1)]:
        barrow.visual(
            mesh_from_geometry(_barrow_mesh(_triangular_plate_geometry(x, 0.018, plate_yz)), f"axle_bracket_{idx}"),
            material=green,
            name=f"axle_bracket_{idx}",
        )
        # Axle nut and two visible mounting bolts on each bracket plate.
        for y, z, rad, length, bolt_idx in [
            (-0.600, 0.250, 0.026, 0.014, 0),
            (-0.680, 0.278, 0.015, 0.010, 1),
            (-0.520, 0.290, 0.015, 0.010, 2),
        ]:
            barrow.visual(
                Cylinder(radius=rad, length=length),
                origin=_barrow_origin(x + (0.014 if x > 0 else -0.014), y, z, (0.0, math.pi / 2.0, 0.0)),
                material=hardware,
                name=f"bracket_bolt_{idx}_{bolt_idx}",
            )

    # Front struts from the tray nose down to the axle bracket region.
    for x, idx in [(-0.27, 0), (0.27, 1)]:
        strut_points = [(x, -0.45, 0.60), (x, -0.55, 0.42), (x, -0.65, 0.28)]
        barrow.visual(_barrow_tube_mesh(strut_points, 0.014, f"front_strut_{idx}"), material=green, name=f"front_strut_{idx}")

    # Wheel child part.  Solid flat-free molded disc wheel on a simple closed
    # hub, centered on the axle and spinning about local X.
    wheel = model.part("wheel")

    polyurethane = model.material("solid_polyurethane", rgba=(0.06, 0.06, 0.05, 1.0))

    # Solid molded disc body — no tread pattern, no sidewall bulge.
    wheel.visual(
        mesh_from_geometry(_solid_disc_wheel_geometry(), "solid_disc"),
        material=polyurethane,
        name="solid_disc",
    )

    # Simple closed hub — smooth cylindrical center insert, no bolt pattern.
    wheel.visual(
        Cylinder(radius=0.050, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green_edge,
        name="closed_hub",
    )

    # Off-axis raised lug on one disc face so wheel spin is visually testable.
    # Slightly embedded into the disc face for connectivity (disc face at X=0.0425).
    wheel.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.042, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="spin_marker",
    )

    model.articulation(
        "axle_pivot_to_barrow",
        ArticulationType.REVOLUTE,
        parent=axle_pivot,
        child=barrow,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=34.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    model.articulation(
        "axle_pivot_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle_pivot,
        child=wheel,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrow = object_model.get_part("barrow")
    wheel = object_model.get_part("wheel")
    body_tip = object_model.get_articulation("axle_pivot_to_barrow")
    wheel_spin = object_model.get_articulation("axle_pivot_to_wheel")

    ctx.check(
        "small class is Single-Wheelbarrow",
        object_model.meta.get("small_class") == "Single-Wheelbarrow",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "agricultural single wheelbarrow has axle pivot, tipping body, and moving wheel",
        len(object_model.parts) == 3
        and body_tip.parent == "wheel_axle_pivot"
        and body_tip.child == "barrow"
        and wheel_spin.parent == "wheel_axle_pivot"
        and wheel_spin.child == "wheel",
        details=f"parts={[p.name for p in object_model.parts]}, articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "front wheel joint spins about axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "barrow body tips forward around the same wheel axle",
        body_tip.articulation_type == ArticulationType.REVOLUTE
        and tuple(body_tip.axis) == (1.0, 0.0, 0.0)
        and body_tip.motion_limits.lower == 0.0
        and body_tip.motion_limits.upper >= 1.0,
        details=f"type={body_tip.articulation_type}, axis={body_tip.axis}, limits={body_tip.motion_limits}",
    )

    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="axle",
        elem_b="closed_hub",
        reason=(
            "The fixed steel axle is intentionally captured inside the closed hub; "
            "this local hidden fit supports the rotating solid disc wheel."
        ),
    )
    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="axle",
        elem_b="solid_disc",
        reason=(
            "The axle passes through the central bore of the solid disc wheel; "
            "this is the intended hub bore fit, not an unintended collision."
        ),
    )

    tray_aabb = ctx.part_element_world_aabb(barrow, elem="tray_shell")
    ctx.check(
        "deep hollow tray proportions",
        tray_aabb is not None
        and tray_aabb[1][0] - tray_aabb[0][0] > 0.80
        and tray_aabb[1][1] - tray_aabb[0][1] > 1.10
        and tray_aabb[1][2] - tray_aabb[0][2] > 0.35,
        details=f"tray_aabb={tray_aabb}",
    )
    for visual_name in (
        "handle_rail_0",
        "handle_rail_1",
        "grip_0",
        "grip_1",
        "front_guard",
        "support_leg_0",
        "support_leg_1",
        "axle",
        "axle_bracket_0",
        "axle_bracket_1",
        "solid_disc",
        "closed_hub",
    ):
        part = wheel if visual_name in {"solid_disc", "closed_hub"} else barrow
        ctx.check(
            f"visible {visual_name} present",
            ctx.part_element_world_aabb(part, elem=visual_name) is not None,
            details=visual_name,
        )

    ctx.expect_within(
        wheel,
        barrow,
        axes="x",
        inner_elem="solid_disc",
        outer_elem="axle",
        margin=0.0,
        name="single wheel is centered between axle ends",
    )
    ctx.expect_overlap(
        wheel,
        barrow,
        axes="x",
        elem_a="closed_hub",
        elem_b="axle",
        min_overlap=0.07,
        name="closed hub surrounds the axle along its width",
    )

    marker0 = ctx.part_element_world_aabb(wheel, elem="spin_marker")
    with ctx.pose({wheel_spin: 1.25}):
        marker1 = ctx.part_element_world_aabb(wheel, elem="spin_marker")

    def _center_yz(aabb):
        return ((aabb[0][1] + aabb[1][1]) * 0.5, (aabb[0][2] + aabb[1][2]) * 0.5)

    moved = False
    if marker0 is not None and marker1 is not None:
        y0, z0 = _center_yz(marker0)
        y1, z1 = _center_yz(marker1)
        moved = abs(y1 - y0) > 0.035 and abs(z1 - z0) > 0.035
    ctx.check("wheel rotation moves spin marker around axle", moved, details=f"rest={marker0}, posed={marker1}")

    def _center_xyz(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    grip0 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    with ctx.pose({body_tip: 0.65}):
        grip1 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    tipped = False
    if grip0 is not None and grip1 is not None:
        rest_center = _center_xyz(grip0)
        tip_center = _center_xyz(grip1)
        tipped = tip_center[2] > rest_center[2] + 0.75 and tip_center[1] < rest_center[1] - 0.45
    ctx.check("body tip joint swings the whole barrow forward around the wheel", tipped, details=f"rest={grip0}, tipped={grip1}")

    return ctx.report()


object_model = build_object_model()
