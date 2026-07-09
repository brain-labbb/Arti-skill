from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
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


def _knee_block_cadquery() -> cq.Workplane:
    """Polycentric prosthetic knee block housing with pivot bosses and tube clamp."""
    # Main housing body: compact shaped block with rounded edges
    body = (
        cq.Workplane("XY")
        .box(0.058, 0.044, 0.050)
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.003)
        .edges("<Z")
        .fillet(0.003)
    )
    # Top mounting flange (wider plate for socket connection)
    top_plate = (
        cq.Workplane("XY")
        .workplane(offset=0.025)
        .rect(0.062, 0.048)
        .extrude(0.006)
        .edges(">Z")
        .fillet(0.002)
    )
    body = body.union(top_plate)

    # Side pivot bosses (cylinders on ±Y faces for knee rotation axis)
    for sign in (1.0, -1.0):
        boss = (
            cq.Workplane("XZ")
            .workplane(offset=sign * 0.022)
            .circle(0.013)
            .extrude(sign * 0.008)
        )
        body = body.union(boss)

    # Pivot bore holes through bosses
    bore = (
        cq.Workplane("XZ")
        .workplane(offset=-0.035)
        .circle(0.006)
        .extrude(0.070)
    )
    body = body.cut(bore)

    # Front weight-reduction recess
    front_recess = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .transformed(offset=(0.0, 0.022, 0.0))
        .rect(0.036, 0.008)
        .extrude(0.034)
        .translate((0.0, 0.0, -0.017))
    )
    body = body.cut(front_recess)

    # Rear weight-reduction recess
    rear_recess = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .transformed(offset=(0.0, -0.022, 0.0))
        .rect(0.030, 0.008)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.014))
    )
    body = body.cut(rear_recess)

    # Bottom tube clamp stub (connects to pylon below, wraps over pylon top collar)
    tube_clamp = (
        cq.Workplane("XY")
        .workplane(offset=-0.025)
        .circle(0.018)
        .extrude(-0.019)
    )
    body = body.union(tube_clamp)

    # Hollow bore through tube clamp
    clamp_bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.022)
        .circle(0.014)
        .extrude(-0.025)
    )
    body = body.cut(clamp_bore)

    # Side link plate detail (polycentric four-bar linkage cue)
    for sign in (1.0, -1.0):
        link = (
            cq.Workplane("XY")
            .workplane(offset=sign * 0.014)
            .transformed(offset=(0.0, sign * 0.016, -0.010))
            .rect(0.012, 0.003)
            .extrude(0.024)
            .translate((0.0, 0.0, -0.012))
        )
        body = body.union(link)

    return body


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
    return body.translate((0.0, 0.0, -0.071))


def _add_ribbon_section(
    geom: MeshGeometry,
    point: tuple[float, float],
    tangent: tuple[float, float],
    width: float,
    thickness: float,
) -> list[int]:
    x, z = point
    tx, tz = tangent
    mag = math.hypot(tx, tz)
    if mag < 1.0e-9:
        tx, tz = 1.0, 0.0
    else:
        tx, tz = tx / mag, tz / mag
    nx, nz = -tz, tx
    return [
        geom.add_vertex(x + nx * thickness * 0.5, -width * 0.5, z + nz * thickness * 0.5),
        geom.add_vertex(x + nx * thickness * 0.5, width * 0.5, z + nz * thickness * 0.5),
        geom.add_vertex(x - nx * thickness * 0.5, width * 0.5, z - nz * thickness * 0.5),
        geom.add_vertex(x - nx * thickness * 0.5, -width * 0.5, z - nz * thickness * 0.5),
    ]


def _add_ribbon(
    geom: MeshGeometry,
    points: list[tuple[float, float]],
    widths: list[float],
    thickness: float,
    *,
    first_section: list[int] | None = None,
) -> list[list[int]]:
    sections: list[list[int]] = []
    for i, point in enumerate(points):
        if i == 0 and first_section is not None:
            sections.append(first_section)
            continue
        if i == 0:
            tangent = (points[1][0] - point[0], points[1][1] - point[1])
        elif i == len(points) - 1:
            tangent = (point[0] - points[i - 1][0], point[1] - points[i - 1][1])
        else:
            tangent = (points[i + 1][0] - points[i - 1][0], points[i + 1][1] - points[i - 1][1])
        sections.append(_add_ribbon_section(geom, point, tangent, widths[i], thickness))

    for j in range(len(sections) - 1):
        for i in range(4):
            _add_quad(geom, sections[j][i], sections[j][(i + 1) % 4], sections[j + 1][(i + 1) % 4], sections[j + 1][i])
    for cap in (sections[0], sections[-1]):
        geom.add_face(cap[0], cap[1], cap[2])
        geom.add_face(cap[0], cap[2], cap[3])
    return sections


def _running_blade_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    # A compact, sculpted C/J carbon foot.  The spring bows rearward just under
    # the ankle, then sweeps forward to a slim, up-curled toe.  This is far
    # shorter and more curved than a full sprint blade, so it reads as an
    # everyday prosthetic foot at a realistic ~0.27 m length rather than the
    # oversized splayed plate it used to be.
    main_points = [
        (0.000, -0.014),
        (0.006, -0.032),
        (0.000, -0.048),
        (-0.018, -0.060),
        (-0.024, -0.070),
        (-0.004, -0.075),
        (0.050, -0.073),
        (0.110, -0.066),
        (0.158, -0.054),
    ]
    main_widths = [0.036, 0.040, 0.044, 0.046, 0.047, 0.048, 0.046, 0.040, 0.032]
    main_sections = _add_ribbon(geom, main_points, main_widths, 0.0070)

    # A short rear heel leaf branches from the lower bend, like a sprint blade's
    # separated heel/spring return.  The first cross-section is shared so the
    # mesh remains one mechanically connected carbon part.
    heel_points = [
        main_points[4],
        (-0.052, -0.070),
        (-0.090, -0.064),
        (-0.116, -0.052),
    ]
    heel_widths = [main_widths[4], 0.044, 0.038, 0.030]
    _add_ribbon(geom, heel_points, heel_widths, 0.0060, first_section=main_sections[4])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trans_femoral_prosthetic_leg")

    socket_mat = Material("beige_rigid_socket", color=(0.72, 0.58, 0.40, 1.0))
    liner_mat = Material("soft_liner_edge", color=(0.92, 0.84, 0.66, 1.0))
    carbon_mat = Material("matte_carbon_black", color=(0.012, 0.014, 0.015, 1.0))
    blade_mat = Material("glossy_carbon_blade", color=(0.006, 0.007, 0.008, 1.0))
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

    knee_joint = model.part("knee_joint")
    knee_joint.visual(
        mesh_from_cadquery(_knee_block_cadquery(), "knee_housing", tolerance=0.0005),
        material=carbon_mat,
        name="knee_housing",
    )
    knee_joint.visual(
        Cylinder(radius=0.007, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="knee_axle",
    )
    knee_joint.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=metal_mat,
        name="knee_clamp_ring",
    )

    pylon = model.part("pylon")
    # Pylon frame at world z = -0.074 via knee joint chain.
    # All local z values preserve original world positions: local_z = world_z + 0.074.
    pylon.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
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
        origin=Origin(xyz=(0.018, 0.000, -0.038)),
        material=blue_mat,
        name="blue_insert_0",
    )
    pylon.visual(
        Box((0.006, 0.032, 0.030)),
        origin=Origin(xyz=(0.018, 0.010, -0.079)),
        material=blue_mat,
        name="blue_insert_1",
    )
    pylon.visual(
        Box((0.006, 0.010, 0.026)),
        origin=Origin(xyz=(0.018, -0.008, -0.116)),
        material=blue_mat,
        name="blue_insert_2",
    )
    pylon.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.119)),
        material=carbon_mat,
        name="round_shin_tube",
    )
    pylon.visual(
        Box((0.042, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=metal_mat,
        name="fork_bridge",
    )
    pylon.visual(
        Box((0.040, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, 0.036, -0.146)),
        material=metal_mat,
        name="fork_plate_0",
    )
    pylon.visual(
        Box((0.040, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.036, -0.146)),
        material=metal_mat,
        name="fork_plate_1",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(_running_blade_geometry(), "curved_carbon_blade"),
        material=blade_mat,
        name="curved_carbon_blade",
    )
    blade.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="ankle_pin",
    )
    blade.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=blade_mat,
        name="blade_neck",
    )

    model.articulation(
        "socket_to_knee_joint",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=knee_joint,
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=3.5, lower=0.0, upper=2.0),
    )
    model.articulation(
        "knee_to_pylon",
        ArticulationType.FIXED,
        parent=knee_joint,
        child=pylon,
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=pylon,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, -0.146)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0, lower=-0.30, upper=0.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    knee_joint = object_model.get_part("knee_joint")
    pylon = object_model.get_part("pylon")
    blade = object_model.get_part("blade")
    knee = object_model.get_articulation("socket_to_knee_joint")
    ankle = object_model.get_articulation("ankle_pitch")

    # Knee joint housing sits below the socket distal plate (contact interface)
    ctx.expect_contact(
        socket,
        knee_joint,
        elem_a="distal_plate",
        elem_b="knee_housing",
        contact_tol=0.002,
        name="knee housing contacts socket distal plate",
    )

    # Knee tube clamp wraps over pylon top collar (intentional tube fit overlap)
    ctx.allow_overlap(
        knee_joint,
        pylon,
        elem_a="knee_housing",
        elem_b="top_collar",
        reason="The knee tube clamp intentionally wraps over the pylon top collar as a clamped tube fit.",
    )
    ctx.expect_within(
        pylon,
        knee_joint,
        axes="xy",
        inner_elem="top_collar",
        outer_elem="knee_housing",
        margin=0.005,
        name="pylon top collar centered within knee housing footprint",
    )
    ctx.expect_contact(
        knee_joint,
        pylon,
        elem_a="knee_clamp_ring",
        elem_b="top_collar",
        contact_tol=0.008,
        name="knee clamp ring seats near pylon top collar",
    )
    ctx.expect_contact(
        blade,
        pylon,
        elem_a="ankle_pin",
        elem_b="fork_plate_0",
        contact_tol=0.0015,
        name="ankle pin touches first fork plate",
    )
    ctx.expect_contact(
        blade,
        pylon,
        elem_a="ankle_pin",
        elem_b="fork_plate_1",
        contact_tol=0.0015,
        name="ankle pin touches second fork plate",
    )

    # Knee joint housing has realistic size
    knee_aabb = ctx.part_element_world_aabb(knee_joint, elem="knee_housing")
    if knee_aabb is not None:
        knee_dx = knee_aabb[1][0] - knee_aabb[0][0]
        knee_dz = knee_aabb[1][2] - knee_aabb[0][2]
        ctx.check(
            "knee_joint housing is a compact polycentric block",
            0.040 <= knee_dx <= 0.080 and 0.060 <= knee_dz <= 0.110,
            details=f"knee_dx={knee_dx:.3f}, knee_dz={knee_dz:.3f}",
        )
    else:
        ctx.fail("knee_joint housing is a compact polycentric block", "missing knee_housing AABB")

    # Knee flexion swings pylon and blade posterior (anatomical knee flexion)
    with ctx.pose({knee: 0.0}):
        straight_blade_pos = ctx.part_world_position(blade)
    with ctx.pose({knee: 1.2}):
        flexed_blade_pos = ctx.part_world_position(blade)
    if straight_blade_pos is not None and flexed_blade_pos is not None:
        knee_shift = sum(
            abs(flexed_blade_pos[i] - straight_blade_pos[i]) for i in range(3)
        )
        ctx.check(
            "socket_to_knee_joint articulation flexes the shin",
            knee_shift > 0.04,
            details=f"blade_shift={knee_shift:.3f} m",
        )
        # Positive knee flexion moves foot posterior (-X direction, anatomical flexion)
        ctx.check(
            "knee positive flexion moves foot posterior",
            flexed_blade_pos[0] < straight_blade_pos[0] - 0.01,
            details=f"straight_x={straight_blade_pos[0]:.3f}, flexed_x={flexed_blade_pos[0]:.3f}",
        )
    else:
        ctx.fail("socket_to_knee_joint articulation flexes the shin", "missing blade position")
        ctx.fail("knee positive flexion moves foot posterior", "missing blade position")

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

    socket_aabb = ctx.part_element_world_aabb(socket, elem="socket_shell")
    blade_aabb = ctx.part_element_world_aabb(blade, elem="curved_carbon_blade")
    if socket_aabb is not None and blade_aabb is not None:
        overall_height = socket_aabb[1][2] - blade_aabb[0][2]
        ctx.check(
            "blade to socket height is prosthetic limb scale (above-knee)",
            0.52 <= overall_height <= 0.62,
            details=f"height={overall_height:.3f} m",
        )
    else:
        ctx.fail("blade to socket height is prosthetic limb scale (above-knee)", "missing socket/blade AABB")

    if blade_aabb is not None:
        ctx.check(
            "carbon foot reaches forward to a toe and back to a heel",
            blade_aabb[1][0] > 0.12 and blade_aabb[0][0] < -0.06,
            details=f"blade_x_span=({blade_aabb[0][0]:.3f}, {blade_aabb[1][0]:.3f})",
        )
        foot_span = blade_aabb[1][0] - blade_aabb[0][0]
        ctx.check(
            "compact curved foot blade with low ground contact",
            0.20 < foot_span < 0.36 and blade_aabb[0][2] < -0.27,
            details=f"foot_span={foot_span:.3f}, min_z={blade_aabb[0][2]:.3f}",
        )
    else:
        ctx.fail("carbon foot reaches forward to a toe and back to a heel", "missing blade AABB")
        ctx.fail("compact curved foot blade with low ground contact", "missing blade AABB")

    with ctx.pose({ankle: -0.30}):
        plantar_blade_aabb = ctx.part_element_world_aabb(blade, elem="curved_carbon_blade")
    with ctx.pose({ankle: 0.35}):
        dorsi_blade_aabb = ctx.part_element_world_aabb(blade, elem="curved_carbon_blade")
    if plantar_blade_aabb is not None and dorsi_blade_aabb is not None:
        shifts = [
            abs(dorsi_blade_aabb[k][a] - plantar_blade_aabb[k][a])
            for k in (0, 1)
            for a in (0, 1, 2)
        ]
        ctx.check(
            "ankle pitch joint swings the carbon foot through its range",
            max(shifts) > 0.025,
            details=f"max_aabb_shift={max(shifts):.3f} m",
        )
    else:
        ctx.fail("ankle pitch joint swings the carbon foot through its range", "missing posed blade AABB")

    return ctx.report()


object_model = build_object_model()
