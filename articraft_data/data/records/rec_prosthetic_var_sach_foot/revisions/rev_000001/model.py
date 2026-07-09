from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftSection,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
)


def _sgn_pow(value: float, power: float) -> float:
    if value == 0.0:
        return 0.0
    return math.copysign(abs(value) ** power, value)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _socket_top_z(theta: float) -> float:
    # Reference-like scalloped brim: a deep front relief, high side wings,
    # and a smaller rear relief.  theta=0 is the forward/front side.
    side_rise = 0.034 * (abs(math.sin(theta)) ** 1.7)
    front_scoop = 0.031 * max(math.cos(theta), 0.0) ** 3.0
    rear_relief = 0.010 * max(-math.cos(theta), 0.0) ** 2.0
    return 0.231 + side_rise - front_scoop - rear_relief


def _socket_surface_point(theta: float, f: float, offset: float = 0.0) -> tuple[float, float, float]:
    exponent = 2.45
    power = 2.0 / exponent
    eased = f**0.82
    rx = 0.034 + (0.073 - 0.034) * eased + offset
    ry = 0.028 + (0.056 - 0.028) * eased + offset
    cx = 0.006 * f
    c = math.cos(theta)
    s = math.sin(theta)
    return (
        cx + rx * _sgn_pow(c, power),
        ry * _sgn_pow(s, power),
        f * _socket_top_z(theta),
    )


def _socket_shell_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    radial_segments = 72
    levels = 10
    thickness = 0.0045
    exponent = 2.45
    power = 2.0 / exponent

    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for j in range(levels + 1):
        f = j / levels
        eased = f**0.82
        rx = 0.034 + (0.073 - 0.034) * eased
        ry = 0.028 + (0.056 - 0.028) * eased
        cx = 0.006 * f
        row: list[int] = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            c = math.cos(theta)
            s = math.sin(theta)
            z = f * _socket_top_z(theta)
            x = cx + rx * _sgn_pow(c, power)
            y = ry * _sgn_pow(s, power)
            row.append(geom.add_vertex(x, y, z))
        outer.append(row)

    for j in range(levels + 1):
        f = j / levels
        eased = f**0.82
        rx = max(0.012, 0.034 + (0.073 - 0.034) * eased - thickness)
        ry = max(0.012, 0.028 + (0.056 - 0.028) * eased - thickness)
        cx = 0.006 * f
        row = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            c = math.cos(theta)
            s = math.sin(theta)
            # The inner bottom is raised, forming a distal cup rather than a
            # straight open tube.
            z = 0.024 + f * (_socket_top_z(theta) - 0.030)
            x = cx + rx * _sgn_pow(c, power)
            y = ry * _sgn_pow(s, power)
            row.append(geom.add_vertex(x, y, z))
        inner.append(row)

    # Outer and inner side walls.
    for j in range(levels):
        for i in range(radial_segments):
            ni = (i + 1) % radial_segments
            _add_quad(geom, outer[j][i], outer[j][ni], outer[j + 1][ni], outer[j + 1][i])
            _add_quad(geom, inner[j + 1][i], inner[j + 1][ni], inner[j][ni], inner[j][i])

    # Thick top lip joining shell and cavity.
    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        _add_quad(geom, outer[-1][i], outer[-1][ni], inner[-1][ni], inner[-1][i])

    # Exterior distal cap and interior cup bottom.
    outer_center = geom.add_vertex(0.0, 0.0, 0.0)
    inner_center = geom.add_vertex(0.0, 0.0, 0.024)
    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        geom.add_face(outer_center, outer[0][ni], outer[0][i])
        geom.add_face(inner_center, inner[0][i], inner[0][ni])

    return geom


def _liner_edge_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    radial_segments = 72
    exponent = 2.45
    power = 2.0 / exponent
    outer_top: list[int] = []
    outer_low: list[int] = []
    inner_top: list[int] = []
    inner_low: list[int] = []
    for i in range(radial_segments):
        theta = 2.0 * math.pi * i / radial_segments
        c = math.cos(theta)
        s = math.sin(theta)
        top_z = _socket_top_z(theta) - 0.002
        low_z = top_z - 0.016
        cx = 0.006
        for ring, rx, ry, z in (
            (outer_top, 0.067, 0.050, top_z),
            (outer_low, 0.067, 0.050, low_z),
            (inner_top, 0.058, 0.042, top_z - 0.001),
            (inner_low, 0.058, 0.042, low_z),
        ):
            ring.append(
                geom.add_vertex(
                    cx + rx * _sgn_pow(c, power),
                    ry * _sgn_pow(s, power),
                    z,
                )
            )

    for i in range(radial_segments):
        ni = (i + 1) % radial_segments
        _add_quad(geom, outer_top[i], outer_top[ni], inner_top[ni], inner_top[i])
        _add_quad(geom, outer_low[ni], outer_low[i], inner_low[i], inner_low[ni])
        _add_quad(geom, outer_top[ni], outer_top[i], outer_low[i], outer_low[ni])
        _add_quad(geom, inner_top[i], inner_top[ni], inner_low[ni], inner_low[i])
    return geom


def _socket_wrap_band_geometry(
    f_lower: float,
    f_upper: float,
    theta_start: float = 0.0,
    theta_end: float = 2.0 * math.pi,
    *,
    radial_segments: int = 72,
) -> MeshGeometry:
    """Thin carbon-fibre wrap laid proud while slightly biting into the shell."""
    geom = MeshGeometry()
    span = theta_end - theta_start
    closed = abs(span - 2.0 * math.pi) < 1.0e-6
    count = radial_segments if closed else max(8, int(radial_segments * abs(span) / (2.0 * math.pi)))
    rows: list[list[int]] = []
    for offset in (0.0022, -0.0014):
        for f in (f_lower, f_upper):
            row = []
            steps = count if closed else count + 1
            for i in range(steps):
                theta = theta_start + span * i / count
                if closed and i == count:
                    theta = theta_start
                row.append(geom.add_vertex(*_socket_surface_point(theta, f, offset=offset)))
            rows.append(row)

    outer_low, outer_high, inner_low, inner_high = rows
    last = count if closed else count
    for i in range(last):
        ni = (i + 1) % count if closed else i + 1
        _add_quad(geom, outer_low[i], outer_low[ni], outer_high[ni], outer_high[i])
        _add_quad(geom, inner_low[ni], inner_low[i], inner_high[i], inner_high[ni])
        _add_quad(geom, outer_high[i], outer_high[ni], inner_high[ni], inner_high[i])
        _add_quad(geom, outer_low[ni], outer_low[i], inner_low[i], inner_low[ni])
    if not closed:
        for i in (0, count):
            _add_quad(geom, outer_low[i], outer_high[i], inner_high[i], inner_low[i])
    return geom


def _pyramid_adapter_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    top_z = -0.044
    bottom_z = -0.062
    top = 0.034
    bottom = 0.024
    top_idx = []
    bottom_idx = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        top_idx.append(geom.add_vertex(0.5 * top * sx, 0.5 * top * sy, top_z))
        bottom_idx.append(geom.add_vertex(0.5 * bottom * sx, 0.5 * bottom * sy, bottom_z))
    for i in range(4):
        ni = (i + 1) % 4
        _add_quad(geom, top_idx[i], top_idx[ni], bottom_idx[ni], bottom_idx[i])
    _add_quad(geom, top_idx[0], top_idx[1], top_idx[2], top_idx[3])
    _add_quad(geom, bottom_idx[3], bottom_idx[2], bottom_idx[1], bottom_idx[0])
    return geom


def _pylon_frame_cadquery() -> cq.Workplane:
    """Sculpted shin pylon with through lightening holes cut from front to rear."""
    body = cq.Workplane("XY").box(0.034, 0.056, 0.122)
    body = body.edges("|Z").fillet(0.004)
    # The workplane on the +X face uses local coordinates approximately as
    # global Y/Z, so these cut-outs become windows through the pylon depth.
    for y, z, w, h in (
        (-0.010, 0.034, 0.021, 0.038),
        (0.010, -0.006, 0.024, 0.035),
        (-0.008, -0.044, 0.019, 0.028),
    ):
        body = (
            body.faces(">X")
            .workplane(centerOption="CenterOfBoundBox")
            .center(y, z)
            .rect(w, h)
            .cutThruAll()
        )
    return body.translate((0.0, 0.0, -0.145))


def _sach_foot_body_geometry() -> MeshGeometry:
    """Solid SACH prosthetic foot shaped like a shoe last.

    The foot body is a lofted D-section form: flat sole on the bottom,
    rounded dome on the top, wider at the ball and narrower at the heel
    and toe.  Cross-sections are defined in the ankle-pivot part frame
    where the origin sits at the ankle pin and +X points toward the toe.
    """
    # Station definitions for the shoe-last shape.
    # (x_pos, half_width, sole_z, top_z)
    stations = [
        (-0.062, 0.024, -0.058, -0.002),  # heel back
        (-0.030, 0.028, -0.060, 0.004),   # heel
        (0.015, 0.025, -0.048, 0.002),    # arch (sole raised)
        (0.070, 0.042, -0.060, -0.002),   # ball (widest)
        (0.110, 0.036, -0.058, -0.006),   # metatarsal
        (0.148, 0.025, -0.050, -0.012),   # toe
        (0.178, 0.013, -0.044, -0.018),   # toe tip
    ]

    n_points = 24
    sections: list[LoftSection] = []

    for x_pos, hw, sole_z, top_z in stations:
        height = top_z - sole_z
        center_z = 0.5 * (sole_z + top_z)
        half_h = 0.5 * height

        points: list[tuple[float, float, float]] = []
        for i in range(n_points):
            theta = 2.0 * math.pi * i / n_points
            c = math.cos(theta)
            s = math.sin(theta)

            # Y: medial-lateral width (symmetric)
            y = hw * c

            # Z: asymmetric profile — flat sole, domed top
            if s <= 0.0:
                # Bottom half: power curve squishes toward the flat sole
                z_norm = -(abs(s) ** 2.5)
                z = max(center_z + z_norm * half_h, sole_z)
            else:
                # Top half: smooth dome
                z = center_z + (s ** 0.8) * half_h

            points.append((x_pos, y, z))

        sections.append(LoftSection(points=tuple(points)))

    spec = SectionLoftSpec(
        sections=tuple(sections),
        cap=True,
        solid=True,
    )
    return section_loft(spec)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trans_tibial_prosthetic_leg")

    socket_mat = Material("beige_rigid_socket", color=(0.72, 0.58, 0.40, 1.0))
    liner_mat = Material("soft_liner_edge", color=(0.92, 0.84, 0.66, 1.0))
    carbon_mat = Material("matte_carbon_black", color=(0.012, 0.014, 0.015, 1.0))
    sach_mat = Material("dark_foam_rubber", color=(0.14, 0.11, 0.09, 1.0))
    metal_mat = Material("brushed_titanium_adapter", color=(0.46, 0.47, 0.45, 1.0))
    blue_mat = Material("blue_accent_inserts", color=(0.00, 0.24, 0.78, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(_socket_shell_geometry(), "socket_shell"),
        material=socket_mat,
        name="socket_shell",
    )
    socket.visual(
        mesh_from_geometry(_liner_edge_geometry(), "liner_edge"),
        material=liner_mat,
        name="liner_edge",
    )
    socket.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=metal_mat,
        name="distal_plate",
    )
    socket.visual(
        mesh_from_geometry(_socket_wrap_band_geometry(0.56, 0.69), "carbon_mid_band"),
        material=carbon_mat,
        name="carbon_mid_band",
    )
    socket.visual(
        mesh_from_geometry(_socket_wrap_band_geometry(0.02, 0.22), "carbon_distal_cup"),
        material=carbon_mat,
        name="carbon_distal_cup",
    )
    socket.visual(
        mesh_from_geometry(
            _socket_wrap_band_geometry(0.22, 0.88, 0.72, 2.0 * math.pi - 0.72),
            "carbon_side_wrap",
        ),
        material=carbon_mat,
        name="carbon_side_wrap",
    )

    adapter = model.part("adapter")
    adapter.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=metal_mat,
        name="top_flange",
    )
    adapter.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=metal_mat,
        name="threaded_stack",
    )
    adapter.visual(
        Cylinder(radius=0.0155, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=metal_mat,
        name="tube_step",
    )
    adapter.visual(
        mesh_from_geometry(_pyramid_adapter_geometry(), "pyramid_coupler"),
        material=metal_mat,
        name="pyramid_coupler",
    )
    adapter.visual(
        Cylinder(radius=0.014, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        material=metal_mat,
        name="lower_plug",
    )

    pylon = model.part("pylon")
    pylon.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.079)),
        material=carbon_mat,
        name="top_collar",
    )
    pylon.visual(
        mesh_from_cadquery(_pylon_frame_cadquery(), "skeletal_shin_frame", tolerance=0.0006),
        material=carbon_mat,
        name="skeletal_shin_frame",
    )
    pylon.visual(
        Box((0.006, 0.010, 0.036)),
        origin=Origin(xyz=(0.018, 0.000, -0.112)),
        material=blue_mat,
        name="blue_insert_0",
    )
    pylon.visual(
        Box((0.006, 0.032, 0.030)),
        origin=Origin(xyz=(0.018, 0.010, -0.153)),
        material=blue_mat,
        name="blue_insert_1",
    )
    pylon.visual(
        Box((0.006, 0.010, 0.026)),
        origin=Origin(xyz=(0.018, -0.008, -0.190)),
        material=blue_mat,
        name="blue_insert_2",
    )
    pylon.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.193)),
        material=carbon_mat,
        name="round_shin_tube",
    )
    pylon.visual(
        Box((0.042, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.188)),
        material=metal_mat,
        name="fork_bridge",
    )
    pylon.visual(
        Box((0.040, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, 0.036, -0.220)),
        material=metal_mat,
        name="fork_plate_0",
    )
    pylon.visual(
        Box((0.040, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.036, -0.220)),
        material=metal_mat,
        name="fork_plate_1",
    )

    sach_foot = model.part("sach_foot")
    sach_foot.visual(
        mesh_from_geometry(_sach_foot_body_geometry(), "sach_foot_body"),
        material=sach_mat,
        name="sach_foot_body",
    )
    sach_foot.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="ankle_pin",
    )

    model.articulation(
        "socket_to_adapter",
        ArticulationType.FIXED,
        parent=socket,
        child=adapter,
        origin=Origin(),
    )
    model.articulation(
        "adapter_to_pylon",
        ArticulationType.FIXED,
        parent=adapter,
        child=pylon,
        origin=Origin(),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=pylon,
        child=sach_foot,
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0, lower=-0.30, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    adapter = object_model.get_part("adapter")
    pylon = object_model.get_part("pylon")
    sach_foot = object_model.get_part("sach_foot")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        socket,
        adapter,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="socket_shell",
        negative_elem="top_flange",
        name="socket shell seats on metal adapter",
    )
    ctx.expect_gap(
        adapter,
        pylon,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0001,
        positive_elem="lower_plug",
        negative_elem="top_collar",
        name="adapter plug seats on shin pylon",
    )
    ctx.expect_contact(
        sach_foot,
        pylon,
        elem_a="ankle_pin",
        elem_b="fork_plate_0",
        contact_tol=0.0015,
        name="ankle pin touches first fork plate",
    )
    ctx.expect_contact(
        sach_foot,
        pylon,
        elem_a="ankle_pin",
        elem_b="fork_plate_1",
        contact_tol=0.0015,
        name="ankle pin touches second fork plate",
    )

    tube_aabb = ctx.part_element_world_aabb(pylon, elem="round_shin_tube")
    if tube_aabb is not None:
        tube_diameter_x = tube_aabb[1][0] - tube_aabb[0][0]
        tube_diameter_y = tube_aabb[1][1] - tube_aabb[0][1]
        ctx.check(
            "round shin tube is about three centimeters diameter",
            0.028 <= tube_diameter_x <= 0.032 and 0.028 <= tube_diameter_y <= 0.032,
            details=f"diameters=({tube_diameter_x:.4f}, {tube_diameter_y:.4f})",
        )
    else:
        ctx.fail("round shin tube is about three centimeters diameter", "missing round_shin_tube AABB")

    # --- SACH foot specific checks ---
    foot_aabb = ctx.part_element_world_aabb(sach_foot, elem="sach_foot_body")
    socket_aabb = ctx.part_element_world_aabb(socket, elem="socket_shell")
    if socket_aabb is not None and foot_aabb is not None:
        overall_height = socket_aabb[1][2] - foot_aabb[0][2]
        ctx.check(
            "sach foot to socket height is prosthetic lower-limb scale",
            0.48 <= overall_height <= 0.58,
            details=f"height={overall_height:.3f} m",
        )
    else:
        ctx.fail("sach foot to socket height is prosthetic lower-limb scale", "missing socket/foot AABB")

    if foot_aabb is not None:
        ctx.check(
            "sach foot has shoe-last shape with heel and toe",
            foot_aabb[1][0] > 0.12 and foot_aabb[0][0] < -0.05,
            details=f"foot_x_span=({foot_aabb[0][0]:.3f}, {foot_aabb[1][0]:.3f})",
        )
        foot_span = foot_aabb[1][0] - foot_aabb[0][0]
        foot_height = foot_aabb[1][2] - foot_aabb[0][2]
        ctx.check(
            "sach foot body is a solid block with realistic proportions",
            0.18 < foot_span < 0.32 and 0.02 < foot_height < 0.08,
            details=f"foot_span={foot_span:.3f}, foot_height={foot_height:.3f}",
        )
    else:
        ctx.fail("sach foot has shoe-last shape with heel and toe", "missing foot AABB")
        ctx.fail("sach foot body is a solid block with realistic proportions", "missing foot AABB")

    # Prove the ankle revolute joint articulates the SACH foot
    with ctx.pose({ankle: -0.30}):
        plantar_aabb = ctx.part_element_world_aabb(sach_foot, elem="sach_foot_body")
    with ctx.pose({ankle: 0.35}):
        dorsi_aabb = ctx.part_element_world_aabb(sach_foot, elem="sach_foot_body")
    if plantar_aabb is not None and dorsi_aabb is not None:
        shifts = [
            abs(dorsi_aabb[k][a] - plantar_aabb[k][a])
            for k in (0, 1)
            for a in (0, 1, 2)
        ]
        ctx.check(
            "ankle pitch joint swings the sach foot through its range",
            max(shifts) > 0.015,
            details=f"max_aabb_shift={max(shifts):.3f} m",
        )
    else:
        ctx.fail("ankle pitch joint swings the sach foot through its range", "missing posed foot AABB")

    return ctx.report()


object_model = build_object_model()
