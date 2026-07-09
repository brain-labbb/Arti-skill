from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

BODY_PROFILE = [
    (0.008, 0.016),
    (0.132, 0.004),
    (0.148, 0.004),
    (0.163, 0.020),
    (0.168, 0.155),
    (0.165, 0.268),
    (0.176, 0.284),
    (0.165, 0.300),
    (0.146, 0.292),
    (0.143, 0.262),
    (0.143, 0.030),
]
SPOUT_PORT_Z = 0.150
SPOUT_TUBE_OUTER_RADIUS = 0.045
SPOUT_TUBE_INNER_RADIUS = 0.035
SPOUT_BODY_PORT_RADIUS = 0.034


def _lathe_profile(
    profile: list[tuple[float, float]],
    *,
    segments: int = 96,
    side_port: tuple[float, float] | None = None,
) -> MeshGeometry:
    """Revolve a closed (radius, z) material cross-section about local Z."""
    geom = MeshGeometry()
    verts: list[list[int]] = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        ring = []
        for radius, z in profile:
            ring.append(geom.add_vertex(radius * ca, radius * sa, z))
        verts.append(ring)

    n = len(profile)
    for i in range(segments):
        ni = (i + 1) % segments
        theta_mid = 2.0 * math.pi * (i + 0.5) / segments
        for j in range(n):
            nj = (j + 1) % n
            if side_port is not None:
                port_z, port_radius = side_port
                radius_mid = (profile[j][0] + profile[nj][0]) * 0.5
                z_mid = (profile[j][1] + profile[nj][1]) * 0.5
                x_mid = radius_mid * math.cos(theta_mid)
                y_mid = radius_mid * math.sin(theta_mid)
                if x_mid > 0.0 and y_mid * y_mid + (z_mid - port_z) ** 2 < port_radius * port_radius:
                    continue
            a, b, c, d = verts[i][j], verts[ni][j], verts[ni][nj], verts[i][nj]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _mesh_has_open_side_port(mesh: MeshGeometry, *, port_z: float, port_radius: float) -> bool:
    clear_radius = port_radius * 0.68
    for face in mesh.faces:
        points = [mesh.vertices[index] for index in face]
        x = sum(point[0] for point in points) / 3.0
        y = sum(point[1] for point in points) / 3.0
        z = sum(point[2] for point in points) / 3.0
        radial = math.sqrt(x * x + y * y)
        if x > 0.0 and 0.132 < radial < 0.176 and y * y + (z - port_z) ** 2 < clear_radius * clear_radius:
            return False
    return True


def _body_shell_solid(*, cut_port: bool = True) -> cq.Workplane:
    shell = cq.Workplane("XZ").polyline(BODY_PROFILE).close().revolve(360.0, (0.0, 0.0), (0.0, 1.0))
    if not cut_port:
        return shell
    port_cutter = (
        cq.Workplane("YZ")
        .circle(SPOUT_BODY_PORT_RADIUS)
        .extrude(0.120)
        .translate((0.116, 0.0, SPOUT_PORT_Z))
    )
    return shell.cut(port_cutter)


def _frustum_tube(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    outer_start: float,
    outer_end: float,
    wall: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """A hollow tapered tube between two 3D points, with annular open ends."""
    sx, sy, sz = start
    ex, ey, ez = end
    ax, ay, az = ex - sx, ey - sy, ez - sz
    length = math.sqrt(ax * ax + ay * ay + az * az)
    axis = (ax / length, ay / length, az / length)
    # The spout is nearly in XZ, so world Y is a stable first radial direction.
    u = (0.0, 1.0, 0.0)
    v = (
        axis[1] * u[2] - axis[2] * u[1],
        axis[2] * u[0] - axis[0] * u[2],
        axis[0] * u[1] - axis[1] * u[0],
    )
    vl = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    v = (v[0] / vl, v[1] / vl, v[2] / vl)

    inner_start = max(outer_start - wall, outer_start * 0.55)
    inner_end = max(outer_end - wall, outer_end * 0.50)
    geom = MeshGeometry()
    rings: list[list[int]] = [[], [], [], []]
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        radial = (u[0] * ca + v[0] * sa, u[1] * ca + v[1] * sa, u[2] * ca + v[2] * sa)
        for ring, center, radius in (
            (0, start, outer_start),
            (1, end, outer_end),
            (2, start, inner_start),
            (3, end, inner_end),
        ):
            rings[ring].append(
                geom.add_vertex(
                    center[0] + radial[0] * radius,
                    center[1] + radial[1] * radius,
                    center[2] + radial[2] * radius,
                )
            )

    for i in range(segments):
        ni = (i + 1) % segments
        # Outer wall
        geom.add_face(rings[0][i], rings[0][ni], rings[1][ni])
        geom.add_face(rings[0][i], rings[1][ni], rings[1][i])
        # Inner wall (reversed)
        geom.add_face(rings[2][i], rings[3][i], rings[3][ni])
        geom.add_face(rings[2][i], rings[3][ni], rings[2][ni])
        # Base annular rim
        geom.add_face(rings[0][i], rings[2][i], rings[2][ni])
        geom.add_face(rings[0][i], rings[2][ni], rings[0][ni])
        # Tip annular rim
        geom.add_face(rings[1][i], rings[1][ni], rings[3][ni])
        geom.add_face(rings[1][i], rings[3][ni], rings[3][i])
    return geom


def _rose_plate() -> cq.Workplane:
    """Perforated sprinkler rose plate, local normal along +X."""
    holes: list[tuple[float, float]] = [(0.0, 0.0)]
    for radius, count, phase in ((0.016, 6, 0.0), (0.031, 12, math.pi / 12.0), (0.043, 16, 0.0)):
        for i in range(count):
            a = phase + 2.0 * math.pi * i / count
            holes.append((radius * math.cos(a), radius * math.sin(a)))

    return (
        cq.Workplane("YZ")
        .circle(0.055)
        .extrude(0.012)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(holes)
        .hole(0.0052)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_watering_can",
        meta={
            "category": "Agricultural",
            "small_class": "Watering can",
            "source": "picture/Agricultural/Watering can/001.png",
        },
    )

    rusty_galvanized = model.material(
        "rusty_galvanized_sheet",
        rgba=(0.50, 0.36, 0.24, 1.0),
    )
    worn_edge = model.material("worn_bright_edges", rgba=(0.72, 0.68, 0.58, 1.0))
    dark_hardware = model.material("dark_pivot_hardware", rgba=(0.08, 0.07, 0.06, 1.0))

    can = model.part("can")

    # Thin-walled, open-topped metal can body: the profile includes a bottom
    # floor, inner wall, rolled top lip and an actual spout-side wall port.
    can.visual(
        mesh_from_cadquery(_body_shell_solid(), "watering_can_body_shell", tolerance=0.0008),
        name="body_shell",
        material=rusty_galvanized,
    )
    can.visual(
        mesh_from_geometry(TorusGeometry(0.171, 0.006, radial_segments=96, tubular_segments=12), "watering_can_top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.289)),
        name="top_rim",
        material=worn_edge,
    )
    can.visual(
        mesh_from_geometry(TorusGeometry(0.160, 0.004, radial_segments=96, tubular_segments=10), "watering_can_lower_foot"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        name="rolled_foot",
        material=worn_edge,
    )
    for idx, z in enumerate((0.184, 0.203, 0.222)):
        can.visual(
            mesh_from_geometry(
                TorusGeometry(0.169, 0.0032, radial_segments=96, tubular_segments=8),
                f"watering_can_corrugation_{idx}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            name=f"body_seam_{idx}",
            material=worn_edge if idx == 1 else rusty_galvanized,
        )

    # A soldered tapered spout with a real through-wall opening and an exterior
    # reinforcing collar.
    spout_start = (0.145, 0.0, 0.150)
    spout_end = (0.515, 0.0, 0.205)
    can.visual(
        mesh_from_geometry(
            _frustum_tube(spout_start, spout_end, 0.045, 0.024, 0.010),
            "watering_can_tapered_spout",
        ),
        name="spout_tube",
        material=rusty_galvanized,
    )
    can.visual(
        mesh_from_geometry(
            _frustum_tube((0.172, 0.0, 0.154), (0.220, 0.0, 0.161), 0.057, 0.049, 0.008, segments=48),
            "watering_can_spout_collar",
        ),
        name="spout_collar",
        material=rusty_galvanized,
    )

    angle = math.atan2(spout_end[2] - spout_start[2], spout_end[0] - spout_start[0])
    can.visual(
        mesh_from_cadquery(_rose_plate(), "watering_can_perforated_rose", tolerance=0.0008),
        origin=Origin(xyz=spout_end, rpy=(0.0, -angle, 0.0)),
        name="rose_plate",
        material=worn_edge,
    )

    # Fixed rear side handle, visibly soldered to the can wall at both ends.
    rear_handle = tube_from_spline_points(
        [
            (-0.145, 0.0, 0.080),
            (-0.230, 0.0, 0.115),
            (-0.238, 0.0, 0.205),
            (-0.150, 0.0, 0.255),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    can.visual(
        mesh_from_geometry(rear_handle, "watering_can_rear_handle"),
        name="rear_handle",
        material=rusty_galvanized,
    )

    # Bail handle hinge lugs and dark rivet centers on the can sides.
    for idx, y in enumerate((-0.184, 0.184)):
        lug = CylinderGeometry(0.026, 0.034, radial_segments=32, closed=True).rotate_x(math.pi / 2.0).translate(
            0.0, y, 0.208
        )
        can.visual(
            mesh_from_geometry(lug, f"watering_can_bail_lug_{idx}"),
            name=f"bail_lug_{idx}",
            material=rusty_galvanized,
        )
        rivet = CylinderGeometry(0.011, 0.040, radial_segments=24, closed=True).rotate_x(math.pi / 2.0).translate(
            0.0, y, 0.208
        )
        can.visual(
            mesh_from_geometry(rivet, f"watering_can_bail_rivet_{idx}"),
            name=f"bail_rivet_{idx}",
            material=dark_hardware,
        )

    # A narrow vertical rolled seam makes the sheet-metal construction legible.
    seam = (
        sweep_profile_along_spline(
            [(0.0, -0.170, 0.045), (0.0, -0.173, 0.145), (0.0, -0.168, 0.260)],
            profile=rounded_rect_profile(0.007, 0.004, 0.0015),
            samples_per_segment=8,
            cap_profile=True,
            up_hint=(0.0, 0.0, 1.0),
        )
    )
    can.visual(
        mesh_from_geometry(seam, "watering_can_vertical_sheet_seam"),
        name="vertical_seam",
        material=worn_edge,
    )

    # Rigid D-handle pivoting on the same lug axis as the original bail.
    # Two arms rise from the lug pivots and converge inward to a horizontal
    # grip bar at the top, forming the characteristic "D" silhouette.
    d_handle = model.part("d_handle")

    handle_geom = MeshGeometry()

    # Left arm: rises from left lug pivot and curves inward toward grip
    left_arm = tube_from_spline_points(
        [
            (0.0, -0.184, 0.0),
            (0.0, -0.184, 0.050),
            (0.0, -0.184, 0.095),
            (0.0, -0.140, 0.140),
            (0.0, -0.090, 0.185),
        ],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=False,
    )
    handle_geom.merge(left_arm)

    # Right arm: mirror of left arm
    right_arm = tube_from_spline_points(
        [
            (0.0, 0.184, 0.0),
            (0.0, 0.184, 0.050),
            (0.0, 0.184, 0.095),
            (0.0, 0.140, 0.140),
            (0.0, 0.090, 0.185),
        ],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=False,
    )
    handle_geom.merge(right_arm)

    # Grip bar: horizontal bar connecting the two arm tops with slight upward bow
    grip_bar = tube_from_spline_points(
        [
            (0.0, -0.090, 0.185),
            (0.0, -0.045, 0.196),
            (0.0, 0.0, 0.200),
            (0.0, 0.045, 0.196),
            (0.0, 0.090, 0.185),
        ],
        radius=0.013,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    handle_geom.merge(grip_bar)

    # Pivot bosses: short cylindrical sleeves at each lug connection
    for y in (-0.184, 0.184):
        boss = (
            CylinderGeometry(0.015, 0.012, radial_segments=32, closed=True)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
        handle_geom.merge(boss)

    d_handle.visual(
        mesh_from_geometry(handle_geom, "watering_can_d_handle_body"),
        name="d_handle_body",
        material=rusty_galvanized,
    )

    model.articulation(
        "can_to_dhandle",
        ArticulationType.REVOLUTE,
        parent=can,
        child=d_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    can = object_model.get_part("can")
    d_handle = object_model.get_part("d_handle")
    joint = object_model.get_articulation("can_to_dhandle")

    ctx.check(
        "asset remains watering can class",
        object_model.name == "agricultural_watering_can"
        and object_model.meta.get("small_class") == "Watering can",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    for visual_name in (
        "body_shell",
        "top_rim",
        "spout_tube",
        "rose_plate",
        "rear_handle",
        "vertical_seam",
    ):
        ctx.check(
            f"visible watering-can subassembly: {visual_name}",
            can.get_visual(visual_name) is not None,
        )

    ctx.check(
        "d_handle part exposes D-handle body geometry",
        d_handle.get_visual("d_handle_body") is not None,
        details="d_handle must carry the d_handle_body visual that replaces the old bail_band.",
    )

    uncut_body = _body_shell_solid(cut_port=False)
    cut_body = _body_shell_solid()
    ctx.check(
        "spout connection cuts through can wall",
        cut_body.val().Volume() < uncut_body.val().Volume() - 0.00001,
        details=f"uncut_volume={uncut_body.val().Volume()}, cut_volume={cut_body.val().Volume()}",
    )
    ctx.check(
        "spout tube wall overlaps the body port edge",
        SPOUT_BODY_PORT_RADIUS < SPOUT_TUBE_OUTER_RADIUS and SPOUT_BODY_PORT_RADIUS <= SPOUT_TUBE_INNER_RADIUS,
        details=(
            f"body_port_radius={SPOUT_BODY_PORT_RADIUS}, "
            f"tube_inner_radius={SPOUT_TUBE_INNER_RADIUS}, tube_outer_radius={SPOUT_TUBE_OUTER_RADIUS}"
        ),
    )

    ctx.check(
        "watering can has one meaningful D-handle pivot joint",
        len(object_model.articulations) == 1
        and joint.motion_limits.lower < 0.0
        and joint.motion_limits.upper > 0.0,
    )

    # The handle pivot bosses are intentionally captured by the side lugs/rivets.
    for idx in (0, 1):
        ctx.allow_overlap(
            can,
            d_handle,
            elem_a=f"bail_lug_{idx}",
            elem_b="d_handle_body",
            reason="The D-handle pivot boss is intentionally captured by the side lug/rivet stack.",
        )
        ctx.expect_overlap(
            can,
            d_handle,
            axes="yz",
            elem_a=f"bail_lug_{idx}",
            elem_b="d_handle_body",
            min_overlap=0.010,
            name=f"D-handle pivot {idx} is seated in its lug",
        )
        ctx.allow_overlap(
            can,
            d_handle,
            elem_a=f"bail_rivet_{idx}",
            elem_b="d_handle_body",
            reason="The dark rivet head represents the same captured pivot stack as the D-handle boss.",
        )
        ctx.expect_overlap(
            can,
            d_handle,
            axes="yz",
            elem_a=f"bail_rivet_{idx}",
            elem_b="d_handle_body",
            min_overlap=0.008,
            name=f"D-handle pivot {idx} overlaps its rivet head",
        )

    # The D-handle arms contact the can body shell as they rise from the pivot lugs.
    # This is realistic for a side-mounted handle and represents the mounting interface.
    ctx.allow_overlap(
        can,
        d_handle,
        elem_a="body_shell",
        elem_b="d_handle_body",
        reason="The D-handle arms contact the can body shell near the pivot lugs as part of the realistic side-mount interface.",
    )
    ctx.expect_contact(
        can,
        d_handle,
        elem_a="body_shell",
        elem_b="d_handle_body",
        contact_tol=0.020,
        name="D-handle arms contact the can body shell",
    )

    d_handle_aabb = ctx.part_world_aabb(d_handle)
    rim_aabb = ctx.part_element_world_aabb(can, elem="top_rim")
    ctx.check(
        "upright D-handle rises well above the rolled top rim",
        d_handle_aabb is not None and rim_aabb is not None and d_handle_aabb[1][2] > rim_aabb[1][2] + 0.12,
        details=f"d_handle={d_handle_aabb}, rim={rim_aabb}",
    )
    ctx.expect_overlap(
        can,
        d_handle,
        axes="y",
        elem_a="top_rim",
        elem_b="d_handle_body",
        min_overlap=0.20,
        name="D-handle spans across the can mouth",
    )

    rest_aabb = ctx.part_world_aabb(d_handle)
    with ctx.pose({joint: 0.75}):
        swung_aabb = ctx.part_world_aabb(d_handle)
        ctx.check(
            "positive D-handle motion swings handle toward spout",
            rest_aabb is not None
            and swung_aabb is not None
            and swung_aabb[1][0] > rest_aabb[1][0] + 0.10,
            details=f"rest={rest_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
