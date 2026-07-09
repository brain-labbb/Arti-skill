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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


TAU = 2.0 * math.pi


def _radial_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    segments: int = 18,
) -> MeshGeometry:
    """Build a slim tube between two points as a mesh cylinder."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("Cylinder endpoints must be separated.")

    geom = MeshGeometry()

    # Stable frame: local z follows the requested segment.
    wz = (dx / length, dy / length, dz / length)
    up = (0.0, 0.0, 1.0) if abs(wz[2]) < 0.92 else (1.0, 0.0, 0.0)
    ux = (
        up[1] * wz[2] - up[2] * wz[1],
        up[2] * wz[0] - up[0] * wz[2],
        up[0] * wz[1] - up[1] * wz[0],
    )
    un = math.sqrt(ux[0] * ux[0] + ux[1] * ux[1] + ux[2] * ux[2])
    ux = (ux[0] / un, ux[1] / un, ux[2] / un)
    uy = (
        wz[1] * ux[2] - wz[2] * ux[1],
        wz[2] * ux[0] - wz[0] * ux[2],
        wz[0] * ux[1] - wz[1] * ux[0],
    )

    center0 = geom.add_vertex(sx, sy, sz)
    center1 = geom.add_vertex(ex, ey, ez)
    ring0: list[int] = []
    ring1: list[int] = []
    for i in range(segments):
        a = TAU * i / segments
        ca, sa = math.cos(a), math.sin(a)
        ox = radius * (ca * ux[0] + sa * uy[0])
        oy = radius * (ca * ux[1] + sa * uy[1])
        oz = radius * (ca * ux[2] + sa * uy[2])
        ring0.append(geom.add_vertex(sx + ox, sy + oy, sz + oz))
        ring1.append(geom.add_vertex(ex + ox, ey + oy, ez + oz))
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(ring0[i], ring0[j], ring1[j])
        geom.add_face(ring0[i], ring1[j], ring1[i])
        geom.add_face(center0, ring0[i], ring0[j])
        geom.add_face(center1, ring1[j], ring1[i])
    return geom


def _lantern_cage_mesh() -> MeshGeometry:
    """Connected cylindrical lattice approximating the lantern's wire guard cage."""
    cage = MeshGeometry()
    radius = 0.055
    n = 96
    z_breaks = [0.038, 0.044, 0.050, 0.107, 0.117, 0.171, 0.181, 0.187]
    verts: dict[tuple[int, int], int] = {}

    def vid(ai: int, zi: int) -> int:
        key = (ai % n, zi)
        if key not in verts:
            a = TAU * (ai % n) / n
            z = z_breaks[zi]
            verts[key] = cage.add_vertex(radius * math.cos(a), radius * math.sin(a), z)
        return verts[key]

    def in_vertical_bar(ai: int) -> bool:
        # Eight slim guards plus slightly wider front/back bars.
        for bar in range(8):
            center = int(round(n * bar / 8.0))
            half = 2 if bar not in (0, 4) else 3
            delta = min((ai - center) % n, (center - ai) % n)
            if delta <= half:
                return True
        return False

    def in_horizontal_band(zi: int) -> bool:
        # Bands occupy the small z intervals [0,1], [3,4], [5,6].
        return zi in (0, 3, 5)

    for ai in range(n):
        for zi in range(len(z_breaks) - 1):
            if in_horizontal_band(zi) or in_vertical_bar(ai):
                a0 = vid(ai, zi)
                a1 = vid(ai + 1, zi)
                b0 = vid(ai, zi + 1)
                b1 = vid(ai + 1, zi + 1)
                cage.add_face(a0, a1, b1)
                cage.add_face(a0, b1, b0)
    return cage


def _vented_base_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.020),
            (0.044, 0.020),
            (0.052, 0.026),
            (0.050, 0.036),
            (0.037, 0.044),
            (0.032, 0.049),
            (0.021, 0.052),
            (0.000, 0.052),
        ],
        segments=72,
    )


def _top_cap_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.000, 0.176),
            (0.053, 0.176),
            (0.063, 0.181),
            (0.062, 0.190),
            (0.053, 0.195),
            (0.026, 0.195),
            (0.018, 0.198),
            (0.018, 0.203),
            (0.013, 0.207),
            (0.000, 0.207),
        ],
        segments=72,
    )


def _leg_mesh() -> MeshGeometry:
    leg = tube_from_spline_points(
        [
            (0.004, 0.0, -0.002),
            (0.018, 0.0, -0.056),
            (0.039, 0.0, -0.111),
            (0.059, 0.0, -0.153),
        ],
        radius=0.0022,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    # Flattened rubber foot, intentionally intersecting the wire end as a crimped cap.
    leg.merge(
        _cylinder_between(
            (0.047, -0.010, -0.154),
            (0.073, 0.010, -0.154),
            radius=0.0032,
            segments=12,
        )
    )
    return leg


def _hook_mesh() -> MeshGeometry:
    """Single J-hook that pivots from the cap center for hanging on a branch or loop."""
    path = [
        (0.000, 0.0, 0.000),
        (0.000, 0.0, 0.014),
        (0.000, 0.0, 0.030),
        (0.001, 0.0, 0.044),
        (0.004, 0.0, 0.054),
        (0.010, 0.0, 0.058),
        (0.016, 0.0, 0.054),
        (0.019, 0.0, 0.044),
        (0.018, 0.0, 0.034),
        (0.015, 0.0, 0.027),
    ]
    return tube_from_spline_points(
        path,
        radius=0.0025,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hanging_tripod_camping_lantern",
        meta={
            "run_notes": (
                "Modeled only the image-referenced camping lantern and three fold-out support legs; "
                "tabletop, food, lighting ambience, and background are intentionally omitted. "
                "The image content appears consistent with the Camping Lantern category."
            )
        },
    )

    dark_metal = model.material("blackened_steel", rgba=(0.005, 0.006, 0.005, 1.0))
    olive_metal = model.material("olive_gray_metal", rgba=(0.30, 0.35, 0.31, 1.0))
    warm_glass = model.material("warm_translucent_glass", rgba=(1.0, 0.54, 0.08, 0.34))
    warm_core = model.material("warm_led_core", rgba=(1.0, 0.73, 0.20, 0.88))
    switch_black = model.material("matte_black_switch", rgba=(0.018, 0.018, 0.016, 1.0))
    white_mark = model.material("white_brand_mark", rgba=(0.93, 0.93, 0.88, 1.0))
    anodized_orange = model.material("anodized_orange_hook", rgba=(0.92, 0.38, 0.06, 1.0))

    body = model.part("lantern_body")

    # Glowing cylindrical diffuser and brighter inner LED tube.
    body.visual(
        Cylinder(radius=0.032, length=0.144),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=warm_glass,
        name="glowing_diffuser",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=warm_core,
        name="led_column",
    )

    body.visual(
        mesh_from_geometry(_lantern_cage_mesh(), "cylindrical_wire_cage"),
        material=dark_metal,
        name="cylindrical_wire_cage",
    )
    body.visual(
        mesh_from_geometry(_vented_base_mesh(), "vented_base"),
        material=olive_metal,
        name="vented_base",
    )
    body.visual(
        mesh_from_geometry(_top_cap_mesh(), "top_vent_cap"),
        material=olive_metal,
        name="top_vent_cap",
    )

    # Raised rectangular control/battery block and switch on the cap.
    body.visual(
        Box((0.068, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.004, 0.206)),
        material=olive_metal,
        name="top_control_block",
    )
    body.visual(
        Box((0.047, 0.004, 0.0012)),
        origin=Origin(xyz=(0.000, -0.020, 0.2155)),
        material=white_mark,
        name="brand_stroke",
    )
    body.visual(
        Box((0.025, 0.016, 0.004)),
        origin=Origin(xyz=(0.025, 0.025, 0.196)),
        material=switch_black,
        name="top_switch",
    )

    # Single central pivot boss on the cap top for the fold-out hook.
    body.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.207), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hook_pivot_boss",
    )

    leg_socket_radius = 0.038
    leg_hinge_z = 0.024
    for i in range(3):
        angle = TAU * i / 3.0 + math.pi / 6.0
        x, y = _radial_xy(leg_socket_radius, angle)
        body.visual(
            Box((0.016, 0.010, 0.010)),
            origin=Origin(xyz=(x, y, leg_hinge_z), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"leg_socket_{i}",
        )

    hook = model.part("top_hook")
    hook.visual(
        mesh_from_geometry(_hook_mesh(), "hook_wire"),
        material=anodized_orange,
        name="hook_wire",
    )
    model.articulation(
        "body_to_hook",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.50),
    )

    leg_mesh = _leg_mesh()
    for i in range(3):
        angle = TAU * i / 3.0 + math.pi / 6.0
        x, y = _radial_xy(leg_socket_radius, angle)
        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_geometry(leg_mesh.copy(), f"foldout_leg_{i}"),
            material=dark_metal,
            name="leg_wire",
        )
        leg.visual(
            Cylinder(radius=0.004, length=0.017),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hinge_pin",
        )
        model.articulation(
            f"body_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=leg,
            origin=Origin(xyz=(x, y, leg_hinge_z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    hook = object_model.get_part("top_hook")
    hook_joint = object_model.get_articulation("body_to_hook")

    # The J-hook wire must sit above the vent cap when standing upright (q=0).
    hook_box = ctx.part_element_world_aabb(hook, elem="hook_wire")
    cap_box = ctx.part_element_world_aabb(body, elem="top_vent_cap")
    ctx.check(
        "upright hook rises above the vent cap",
        hook_box is not None and cap_box is not None and hook_box[1][2] > cap_box[1][2] + 0.020,
        details=f"hook_box={hook_box}, cap_box={cap_box}",
    )
    ctx.expect_overlap(
        body,
        hook,
        axes="z",
        elem_a="hook_pivot_boss",
        elem_b="hook_wire",
        min_overlap=0.001,
        name="hook wire overlaps the pivot boss on Z",
    )

    # Folding the hook toward the body should swing its tip in the +X direction
    # (axis is +Y, so positive q rotates local +Z toward +X via right-hand rule).
    with ctx.pose({hook_joint: 1.0}):
        folded_hook_box = ctx.part_element_world_aabb(hook, elem="hook_wire")
    ctx.check(
        "body_to_hook hinge folds the J-hook forward",
        hook_box is not None
        and folded_hook_box is not None
        and folded_hook_box[1][0] > hook_box[1][0] + 0.010,
        details=f"upright={hook_box}, folded={folded_hook_box}",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        joint = object_model.get_articulation(f"body_to_leg_{i}")
        ctx.allow_overlap(
            body,
            leg,
            elem_a=f"leg_socket_{i}",
            elem_b="hinge_pin",
            reason="The slim leg hinge pin is intentionally captured inside the socket block at the lantern base.",
        )
        ctx.expect_contact(
            leg,
            body,
            elem_a="hinge_pin",
            elem_b=f"leg_socket_{i}",
            contact_tol=0.003,
            name=f"leg_{i} hinge sits in its socket",
        )
        leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
        base_box = ctx.part_element_world_aabb(body, elem="vented_base")
        ctx.check(
            f"leg_{i} extends below the lantern base",
            leg_box is not None and base_box is not None and leg_box[0][2] < base_box[0][2] - 0.10,
            details=f"leg_box={leg_box}, base_box={base_box}",
        )

        rest_leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
        with ctx.pose({joint: 1.0}):
            folded_leg_box = ctx.part_element_world_aabb(leg, elem="leg_wire")
            ctx.expect_contact(
                leg,
                body,
                elem_a="hinge_pin",
                elem_b=f"leg_socket_{i}",
                contact_tol=0.003,
                name=f"leg_{i} remains hinged while folding",
            )
        ctx.check(
            f"leg_{i} folds upward about its base hinge",
            rest_leg_box is not None
            and folded_leg_box is not None
            and folded_leg_box[0][2] > rest_leg_box[0][2] + 0.060,
            details=f"rest={rest_leg_box}, folded={folded_leg_box}",
        )

    return ctx.report()


object_model = build_object_model()
