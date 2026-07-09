from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


MOUSE_LENGTH = 0.114
MOUSE_WIDTH = 0.057
BODY_HALF_LENGTH = MOUSE_LENGTH * 0.5
BODY_HALF_WIDTH = MOUSE_WIDTH * 0.5
RIM_Z = 0.0065
DOME_HEIGHT = 0.0152


def _superellipse_point(a: float, b: float, exponent: float, theta: float) -> tuple[float, float]:
    c = cos(theta)
    s = sin(theta)
    power = 2.0 / exponent
    x = a * (1.0 if c >= 0.0 else -1.0) * (abs(c) ** power)
    y = b * (1.0 if s >= 0.0 else -1.0) * (abs(s) ** power)
    return x, y


def _top_shell_z(x: float, y: float) -> float:
    """Height of the continuous Magic-Mouse-like top shell at a top-view point."""
    nx = abs(x) / BODY_HALF_LENGTH
    ny = abs(y) / BODY_HALF_WIDTH
    radial = min(1.0, (nx**3.15 + ny**3.15) ** (1.0 / 3.15))
    return RIM_Z + DOME_HEIGHT * ((1.0 - radial**2) ** 0.58)


def _build_mouse_body_mesh() -> MeshGeometry:
    """Single closed, low, elongated superellipse dome with rounded underside."""
    geom = MeshGeometry()
    segments = 112
    exponent = 3.15

    def add_ring(scale: float, z: float | None = None) -> list[int]:
        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * pi * i / segments
            x, y = _superellipse_point(BODY_HALF_LENGTH * scale, BODY_HALF_WIDTH * scale, exponent, theta)
            ring.append(geom.add_vertex(x, y, _top_shell_z(x, y) if z is None else z))
        return ring

    center_id = geom.add_vertex(0.0, 0.0, _top_shell_z(0.0, 0.0))
    top_rings: list[list[int]] = []
    for scale in (0.10, 0.22, 0.34, 0.46, 0.58, 0.70, 0.82, 0.92, 1.00):
        top_rings.append(add_ring(scale))

    first_ring = top_rings[0]
    for i in range(segments):
        geom.add_face(center_id, first_ring[i], first_ring[(i + 1) % segments])

    previous = first_ring
    for ring in top_rings[1:]:
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(previous[i], ring[i], ring[j])
            geom.add_face(previous[i], ring[j], previous[j])
        previous = ring

    # Rounded lower side and the white plastic base are connected to the same
    # shell mesh, avoiding a visible seam or any floating subpiece.
    side_rings = [
        add_ring(0.995, 0.0043),
        add_ring(0.970, 0.0016),
        add_ring(0.930, 0.0000),
    ]
    for ring in side_rings:
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(previous[i], ring[i], ring[j])
            geom.add_face(previous[i], ring[j], previous[j])
        previous = ring

    bottom_center = geom.add_vertex(0.0, 0.0, 0.0)
    for i in range(segments):
        geom.add_face(previous[(i + 1) % segments], previous[i], bottom_center)

    return geom


def _build_logo_mesh(center_x: float) -> MeshGeometry:
    """A tiny grey bitten-apple style inlay that follows the top curve."""
    geom = MeshGeometry()
    # Local x is mouse length; local y is mouse width.  The leaf is joined by a
    # narrow stem so the logo is one supported, connected decal mesh.
    profile = [
        (-0.0044, -0.0001),
        (-0.0038, -0.0022),
        (-0.0024, -0.0035),
        (-0.0006, -0.0039),
        (0.0010, -0.0032),
        (0.0022, -0.0020),
        (0.0030, -0.0008),
        (0.0042, -0.0014),
        (0.0056, -0.0009),
        (0.0060, 0.0003),
        (0.0047, 0.0011),
        (0.0032, 0.0006),
        (0.0025, 0.0011),
        (0.0018, 0.0009),
        (0.0028, 0.0020),
        (0.0023, 0.0030),
        (0.0009, 0.0035),
        (-0.0005, 0.0026),
        (-0.0017, 0.0033),
        (-0.0033, 0.0029),
        (-0.0042, 0.0015),
    ]
    # The decal is a very shallow inlay: it sits almost flush with the glossy
    # shell and is minutely embedded so it is physically seated, not floating.
    center_z = _top_shell_z(center_x, 0.0)
    top_ids: list[int] = []
    bottom_ids: list[int] = []
    for x, y in profile:
        world_x = center_x + x
        shell_z = _top_shell_z(world_x, y)
        top_ids.append(geom.add_vertex(x, y, shell_z + 0.00004 - center_z))
        bottom_ids.append(geom.add_vertex(x, y, shell_z - 0.00016 - center_z))
    top_center = geom.add_vertex(0.0, 0.0, 0.00004)
    bottom_center = geom.add_vertex(0.0, 0.0, -0.00016)
    for i in range(len(top_ids)):
        j = (i + 1) % len(top_ids)
        geom.add_face(top_center, top_ids[i], top_ids[j])
        geom.add_face(bottom_ids[j], bottom_ids[i], bottom_center)
        geom.add_face(top_ids[i], bottom_ids[i], bottom_ids[j])
        geom.add_face(top_ids[i], bottom_ids[j], top_ids[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apple_magic_mouse")

    glossy_white = model.material("glossy_white", rgba=(0.96, 0.97, 0.97, 1.0))
    base_white = model.material("base_white", rgba=(0.91, 0.92, 0.91, 1.0))
    glide_plastic = model.material("glide_plastic", rgba=(0.78, 0.80, 0.80, 1.0))
    pale_logo = model.material("pale_logo", rgba=(0.68, 0.70, 0.72, 1.0))

    body = model.part("mouse_body")
    body.visual(
        mesh_from_geometry(_build_mouse_body_mesh(), "magic_mouse_smooth_body"),
        material=glossy_white,
        name="smooth_top_shell",
    )
    # A very thin off-white lower plate gives the underside/base read while the
    # main body remains a seamless one-piece white top.
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(superellipse_profile(0.100, 0.047, exponent=3.6, segments=72), 0.0007, center=True),
            "magic_mouse_white_base",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.00035)),
        material=base_white,
        name="white_plastic_base",
    )
    body.inertial = Inertial.from_geometry(
        Box((MOUSE_LENGTH, MOUSE_WIDTH, RIM_Z + DOME_HEIGHT)),
        mass=0.099,
        origin=Origin(xyz=(0.0, 0.0, (RIM_Z + DOME_HEIGHT) * 0.45)),
    )

    logo_center_x = -0.022
    logo = model.part("logo")
    logo.visual(
        mesh_from_geometry(_build_logo_mesh(logo_center_x), "magic_mouse_apple_logo"),
        material=pale_logo,
        name="apple_logo_decal",
    )

    glide_profile = superellipse_profile(0.088, 0.0060, exponent=4.0, segments=48)
    glide_mesh = mesh_from_geometry(
        ExtrudeGeometry(glide_profile, 0.0008, center=True),
        "magic_mouse_side_glide",
    )
    for index, y in enumerate((-0.0205, 0.0205)):
        glide = model.part(f"glide_{index}")
        glide.visual(glide_mesh, material=glide_plastic, name="thin_glide")
        glide.inertial = Inertial.from_geometry(
            Box((0.088, 0.0060, 0.0008)),
            mass=0.002,
        )
        model.articulation(
            f"body_to_glide_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=glide,
            origin=Origin(xyz=(0.0, y, -0.0004)),
        )

    model.articulation(
        "body_to_logo",
        ArticulationType.FIXED,
        parent=body,
        child=logo,
        origin=Origin(xyz=(logo_center_x, 0.0, _top_shell_z(logo_center_x, 0.0))),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("mouse_body")
    logo = object_model.get_part("logo")
    glide_0 = object_model.get_part("glide_0")
    glide_1 = object_model.get_part("glide_1")

    ctx.allow_overlap(
        body,
        logo,
        elem_a="smooth_top_shell",
        elem_b="apple_logo_decal",
        reason="The printed Apple logo is modeled as a nearly flush inlay with a minute seated embed into the curved shell.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        mins, maxs = body_aabb
        dims = (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])
        ctx.check(
            "low elongated Magic Mouse proportions",
            0.106 <= dims[0] <= 0.122 and 0.052 <= dims[1] <= 0.062 and 0.018 <= dims[2] <= 0.026,
            details=f"body dimensions were {dims}",
        )
    else:
        ctx.fail("low elongated Magic Mouse proportions", "body AABB unavailable")

    ctx.check(
        "no separate mechanical buttons",
        all(j.articulation_type == ArticulationType.FIXED for j in object_model.articulations),
        details="Magic Mouse top should be a single touch surface, so authored joints are fixed mounts only.",
    )
    ctx.expect_within(logo, body, axes="xy", margin=0.0, name="small logo lies on the top footprint")
    ctx.expect_overlap(
        logo,
        body,
        axes="xy",
        min_overlap=0.004,
        elem_a="apple_logo_decal",
        elem_b="smooth_top_shell",
        name="logo inlay is seated on the shell area",
    )
    logo_aabb = ctx.part_world_aabb(logo)
    ctx.check(
        "Apple logo is near the rear half",
        logo_aabb is not None and 0.004 <= (logo_aabb[1][0] - logo_aabb[0][0]) <= 0.014 and (logo_aabb[0][0] + logo_aabb[1][0]) * 0.5 < -0.012,
        details=f"logo_aabb={logo_aabb}",
    )
    for glide in (glide_0, glide_1):
        ctx.expect_within(glide, body, axes="xy", margin=0.0, name=f"{glide.name} stays inside underside footprint")
        ctx.expect_gap(
            body,
            glide,
            axis="z",
            max_gap=0.00025,
            max_penetration=0.00005,
            name=f"{glide.name} is seated against the underside",
        )

    return ctx.report()


object_model = build_object_model()
