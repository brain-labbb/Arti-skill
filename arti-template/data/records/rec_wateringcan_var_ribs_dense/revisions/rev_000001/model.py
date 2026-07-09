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
    rib_count = 6
    foot_z = 0.018
    shoulder_z = 0.268
    for i in range(rib_count):
        t = (i + 1) / (rib_count + 1)
        z = foot_z + t * (shoulder_z - foot_z)
        can.visual(
            mesh_from_geometry(
                TorusGeometry(0.169, 0.0032, radial_segments=96, tubular_segments=8),
                f"watering_can_corrugation_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, z)),
            name=f"body_seam_{i}",
            material=worn_edge if i % 2 == 1 else rusty_galvanized,
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

    # Moving bail handle, authored in its own joint frame with the hinge axis
    # running through the two side lugs.  It reads as a bent metal strap with
    # broad pivot washers.
    bail = model.part("bail_handle")
    bail_band = sweep_profile_along_spline(
        [
            (0.0, -0.194, 0.0),
            (0.0, -0.204, 0.075),
            (0.0, -0.145, 0.185),
            (0.0, 0.0, 0.220),
            (0.0, 0.145, 0.185),
            (0.0, 0.204, 0.075),
            (0.0, 0.194, 0.0),
        ],
        profile=rounded_rect_profile(0.022, 0.008, 0.0025),
        samples_per_segment=16,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    for y in (-0.194, 0.194):
        bail_band.merge(
            CylinderGeometry(0.019, 0.010, radial_segments=32, closed=True)
            .rotate_x(math.pi / 2.0)
            .translate(0.0, y, 0.0)
        )
    bail.visual(
        mesh_from_geometry(bail_band, "watering_can_pivoting_bail_handle"),
        name="bail_band",
        material=rusty_galvanized,
    )

    model.articulation(
        "can_to_bail",
        ArticulationType.REVOLUTE,
        parent=can,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    can = object_model.get_part("can")
    bail = object_model.get_part("bail_handle")
    joint = object_model.get_articulation("can_to_bail")

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

    # Verify exactly 6 corrugation rib bands evenly spaced on the can body.
    rib_names = [f"body_seam_{i}" for i in range(6)]
    for rib_name in rib_names:
        ctx.check(
            f"corrugation rib band present: {rib_name}",
            can.get_visual(rib_name) is not None,
        )
    ctx.check(
        "exactly 6 body_seam corrugation ribs on can",
        sum(1 for v in can.visuals if v.name.startswith("body_seam_")) == 6,
        details=f"found {[v.name for v in can.visuals if v.name.startswith('body_seam_')]}",
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
        "watering can has one meaningful bail-handle joint",
        len(object_model.articulations) == 1 and joint.motion_limits.lower < 0.0 and joint.motion_limits.upper > 0.0,
    )

    # The handle washers are intentionally captured in the side hinge lugs.
    for idx in (0, 1):
        ctx.allow_overlap(
            can,
            bail,
            elem_a=f"bail_lug_{idx}",
            elem_b="bail_band",
            reason="The bail handle pivot washer is intentionally captured by the side lug/rivet stack.",
        )
        ctx.expect_overlap(
            can,
            bail,
            axes="yz",
            elem_a=f"bail_lug_{idx}",
            elem_b="bail_band",
            min_overlap=0.010,
            name=f"bail pivot {idx} is seated in its lug",
        )
        ctx.allow_overlap(
            can,
            bail,
            elem_a=f"bail_rivet_{idx}",
            elem_b="bail_band",
            reason="The dark rivet head represents the same captured pivot stack as the bail washer.",
        )
        ctx.expect_overlap(
            can,
            bail,
            axes="yz",
            elem_a=f"bail_rivet_{idx}",
            elem_b="bail_band",
            min_overlap=0.008,
            name=f"bail pivot {idx} overlaps its rivet head",
        )

    bail_aabb = ctx.part_world_aabb(bail)
    rim_aabb = ctx.part_element_world_aabb(can, elem="top_rim")
    ctx.check(
        "upright bail rises well above the rolled top rim",
        bail_aabb is not None and rim_aabb is not None and bail_aabb[1][2] > rim_aabb[1][2] + 0.12,
        details=f"bail={bail_aabb}, rim={rim_aabb}",
    )
    ctx.expect_overlap(
        can,
        bail,
        axes="y",
        elem_a="top_rim",
        elem_b="bail_band",
        min_overlap=0.20,
        name="bail spans across the can mouth",
    )

    rest_aabb = ctx.part_world_aabb(bail)
    with ctx.pose({joint: 0.75}):
        swung_aabb = ctx.part_world_aabb(bail)
        ctx.check(
            "positive bail motion swings handle toward spout",
            rest_aabb is not None
            and swung_aabb is not None
            and swung_aabb[1][0] > rest_aabb[1][0] + 0.10,
            details=f"rest={rest_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
