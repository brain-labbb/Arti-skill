from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _triangle_area(a, b, c) -> float:
    ax, ay, az = a
    bx, by, bz = b
    cx, cy, cz = c
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    return 0.5 * math.sqrt(
        (uy * vz - uz * vy) ** 2
        + (uz * vx - ux * vz) ** 2
        + (ux * vy - uy * vx) ** 2
    )


def _add_tri(geom: MeshGeometry, a, b, c) -> None:
    if _triangle_area(a, b, c) < 1.0e-10:
        return
    ia = geom.add_vertex(*a)
    ib = geom.add_vertex(*b)
    ic = geom.add_vertex(*c)
    geom.add_face(ia, ib, ic)


def _add_quad(geom: MeshGeometry, a, b, c, d) -> None:
    _add_tri(geom, a, b, c)
    _add_tri(geom, a, c, d)


def _build_shuttle_body_mesh() -> MeshGeometry:
    """Open, carved, boat-shaped loom shuttle body in meters.

    X is the long shuttle axis.  The mesh is intentionally a hollow trough:
    the top deck has a long central opening, and the opening has visible inner
    walls and a floor so the bobbin reads as seated inside the wooden body.
    """

    length = 0.320
    width = 0.056
    opening_half_len = 0.107
    opening_half_width = 0.018
    floor_z = 0.004

    geom = MeshGeometry()
    # Odd count puts one section at the widest middle of the shuttle.
    xs = [-length / 2 + length * i / 40 for i in range(41)]

    def s_at(x: float) -> float:
        return max(0.0, min(1.0, (x + length / 2) / length))

    def fullness(x: float) -> float:
        # Broad mid-body with sharp, boat-like taper to the nose and tail.
        return max(0.035, math.sin(math.pi * s_at(x)) ** 0.58)

    def outer_half_width(x: float) -> float:
        return (width / 2) * fullness(x)

    def top_z(x: float) -> float:
        # Slight crown through the middle, falling toward the pointed ends.
        return 0.022 + 0.012 * (math.sin(math.pi * s_at(x)) ** 0.75)

    def chine_z(x: float) -> float:
        return 0.007 + 0.006 * (math.sin(math.pi * s_at(x)) ** 0.9)

    def keel_z(x: float) -> float:
        return -0.005 + 0.002 * (math.sin(math.pi * s_at(x)) ** 0.9)

    def inner_half_width(x: float) -> float:
        u = abs(x) / opening_half_len
        if u >= 1.0:
            return 0.0
        return opening_half_width * ((1.0 - u**4) ** 0.5)

    # Curved exterior hull strips, from left rim down over the keel to right rim.
    exterior_sections = []
    for x in xs:
        w = outer_half_width(x)
        zt = top_z(x)
        zc = chine_z(x)
        zk = keel_z(x)
        exterior_sections.append(
            [
                (x, -w, zt),
                (x, -0.96 * w, zc),
                (x, -0.52 * w, zk),
                (x, 0.0, zk - 0.002),
                (x, 0.52 * w, zk),
                (x, 0.96 * w, zc),
                (x, w, zt),
            ]
        )

    for i in range(len(exterior_sections) - 1):
        asec = exterior_sections[i]
        bsec = exterior_sections[i + 1]
        for j in range(len(asec) - 1):
            _add_quad(geom, asec[j], bsec[j], bsec[j + 1], asec[j + 1])

    # Close the tiny nose and tail sections so the body reads as pointed wood.
    for sec in (exterior_sections[0], exterior_sections[-1]):
        mid = (sec[0][0], 0.0, sum(p[2] for p in sec) / len(sec))
        for j in range(len(sec) - 1):
            _add_tri(geom, mid, sec[j], sec[j + 1])

    # Top deck around the central bobbin slot: two rim strips leave a genuine
    # opening in the middle.  Where inner width collapses, the deck becomes a
    # closed pointed end.
    for i in range(len(xs) - 1):
        x0, x1 = xs[i], xs[i + 1]
        wo0, wo1 = outer_half_width(x0), outer_half_width(x1)
        wi0, wi1 = inner_half_width(x0), inner_half_width(x1)
        zt0, zt1 = top_z(x0), top_z(x1)
        # Port/left deck strip.
        _add_quad(
            geom,
            (x0, -wo0, zt0),
            (x1, -wo1, zt1),
            (x1, -wi1, zt1 - 0.001),
            (x0, -wi0, zt0 - 0.001),
        )
        # Starboard/right deck strip.
        _add_quad(
            geom,
            (x0, wi0, zt0 - 0.001),
            (x1, wi1, zt1 - 0.001),
            (x1, wo1, zt1),
            (x0, wo0, zt0),
        )

        # Smooth carved inner walls and the bottom floor of the hollow.  The
        # end sections degenerate to a rounded slot end rather than a square cut.
        if wi0 > 0.001 and wi1 > 0.001:
            _add_quad(
                geom,
                (x0, -wi0, zt0 - 0.001),
                (x1, -wi1, zt1 - 0.001),
                (x1, -wi1 * 0.86, floor_z),
                (x0, -wi0 * 0.86, floor_z),
            )
            _add_quad(
                geom,
                (x0, wi0 * 0.86, floor_z),
                (x1, wi1 * 0.86, floor_z),
                (x1, wi1, zt1 - 0.001),
                (x0, wi0, zt0 - 0.001),
            )
            _add_quad(
                geom,
                (x0, -wi0 * 0.86, floor_z),
                (x1, -wi1 * 0.86, floor_z),
                (x1, wi1 * 0.86, floor_z),
                (x0, wi0 * 0.86, floor_z),
            )

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wooden_loom_shuttle",
        meta={
            "category": "Textiles_Fabric / Loom shuttle",
            "description": "Elongated wooden loom shuttle with hollow bobbin cavity and a revolute bobbin.",
        },
    )

    wood = model.material("smooth_worn_honey_wood", rgba=(0.72, 0.38, 0.14, 1.0))
    end_grain = model.material("darker_worn_end_grain", rgba=(0.43, 0.22, 0.09, 1.0))
    pale_wood = model.material("pale_bobbin_wood", rgba=(0.86, 0.65, 0.36, 1.0))
    thread = model.material("cream_thread", rgba=(0.93, 0.82, 0.60, 1.0))
    thread_shadow = model.material("wound_thread_shadow", rgba=(0.70, 0.52, 0.32, 1.0))
    brass = model.material("worn_brass", rgba=(0.84, 0.63, 0.28, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_shuttle_body_mesh(), "carved_hollow_body"),
        material=wood,
        name="body_shell",
    )
    # Small darker inlaid wear streaks sit slightly into the top deck; they make
    # the body read as smooth used wood without becoming separate props.
    for idx, y in enumerate((-0.022, 0.022)):
        grain = TorusGeometry(radius=0.060, tube=0.00055, radial_segments=8, tubular_segments=40)
        grain.scale(1.0, 0.10, 0.02).translate(0.0, y, 0.030)
        body.visual(
            mesh_from_geometry(grain, f"long_wear_line_{idx}"),
            material=end_grain,
            name=f"wear_line_{idx}",
        )
    for idx, x in enumerate((-0.158, 0.158)):
        tip = SphereGeometry(0.0055, width_segments=24, height_segments=12)
        tip.scale(1.0, 0.72, 0.72).translate(x, 0.0, 0.018)
        body.visual(
            mesh_from_geometry(tip, f"metal_tip_{idx}"),
            material=brass,
            name=f"metal_tip_{idx}",
        )
    # Two small bearing blocks are carved up from the cavity floor.  They
    # lightly capture the bobbin axle so the spool is mechanically supported,
    # not floating in the hollow.
    for idx, x in enumerate((-0.071, 0.071)):
        body.visual(
            Box((0.007, 0.006, 0.0114)),
            origin=Origin(xyz=(x, 0.0, 0.0096)),
            material=wood,
            name=f"bearing_{idx}",
        )

    bobbin = model.part("bobbin")
    # A brass axle passes through a cream/thread-wrapped spool.  All cylinders
    # are aligned to the shuttle's long X axis.
    x_axis = Origin(rpy=(0.0, math.pi / 2, 0.0))
    bobbin.visual(
        Cylinder(radius=0.003, length=0.146),
        origin=x_axis,
        material=brass,
        name="axle",
    )
    bobbin.visual(
        Cylinder(radius=0.0075, length=0.108),
        origin=x_axis,
        material=thread,
        name="thread_core",
    )
    for idx, x in enumerate((-0.058, 0.058)):
        bobbin.visual(
            Cylinder(radius=0.0115, length=0.007),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=pale_wood,
            name=f"flange_{idx}",
        )
    for idx, x in enumerate((-0.036, -0.020, -0.004, 0.012, 0.028, 0.044)):
        bobbin.visual(
            Cylinder(radius=0.0079, length=0.0014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=thread_shadow,
            name=f"thread_band_{idx}",
        )

    model.articulation(
        "body_to_bobbin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bobbin,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bobbin = object_model.get_part("bobbin")
    joint = object_model.get_articulation("body_to_bobbin")

    body_aabb = ctx.part_world_aabb(body)
    bobbin_aabb = ctx.part_world_aabb(bobbin)
    if body_aabb is not None:
        bmin, bmax = body_aabb
        length = bmax[0] - bmin[0]
        width = bmax[1] - bmin[1]
        height = bmax[2] - bmin[2]
        ctx.check(
            "body has elongated shuttle proportions",
            length > 0.28 and width < 0.075 and height < 0.055 and length / max(width, 1e-6) > 4.0,
            details=f"length={length:.3f}, width={width:.3f}, height={height:.3f}",
        )

    ctx.expect_within(
        bobbin,
        body,
        axes="xy",
        margin=0.002,
        name="bobbin sits inside the hollow shuttle footprint",
    )
    ctx.expect_overlap(
        bobbin,
        body,
        axes="x",
        min_overlap=0.10,
        name="bobbin spans the central cavity length",
    )
    for idx in (0, 1):
        ctx.allow_overlap(
            body,
            bobbin,
            elem_a=f"bearing_{idx}",
            elem_b="axle",
            reason="The bobbin axle is intentionally lightly seated in the carved bearing block.",
        )
        ctx.expect_gap(
            bobbin,
            body,
            axis="z",
            positive_elem="axle",
            negative_elem=f"bearing_{idx}",
            max_gap=0.001,
            max_penetration=0.0006,
            name=f"axle seated on bearing {idx}",
        )

    if body_aabb is not None and bobbin_aabb is not None:
        bmin, bmax = body_aabb
        smin, smax = bobbin_aabb
        ctx.check(
            "bobbin is visible above the carved cavity floor",
            smin[2] > bmin[2] + 0.008 and smax[2] < bmax[2] + 0.002,
            details=f"body_z=({bmin[2]:.3f},{bmax[2]:.3f}), bobbin_z=({smin[2]:.3f},{smax[2]:.3f})",
        )

    rest_pos = ctx.part_world_position(bobbin)
    with ctx.pose({joint: math.pi / 2}):
        spun_pos = ctx.part_world_position(bobbin)
        ctx.expect_within(
            bobbin,
            body,
            axes="xy",
            margin=0.002,
            name="rotated bobbin remains retained in cavity",
        )
    ctx.check(
        "bobbin revolute joint spins about its axle",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
