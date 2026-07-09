from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_tube(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _arc_points(
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    y: float = 0.0,
    z_offset: float = 0.0,
    samples: int = 32,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    for i in range(samples + 1):
        t = math.radians(start_deg + (end_deg - start_deg) * i / samples)
        pts.append((radius * math.cos(t), y, radius * math.sin(t) + z_offset))
    return pts


def _annular_sector_geometry(
    inner_radius: float,
    outer_radius: float,
    width: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 56,
    z_offset: float = 0.0,
) -> MeshGeometry:
    """Thick annular sector in local XZ, extruded along local Y."""
    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []
    half = width * 0.5
    for i in range(segments + 1):
        theta = math.radians(start_deg + (end_deg - start_deg) * i / segments)
        c = math.cos(theta)
        s = math.sin(theta)
        # outer/back, inner/back, outer/front, inner/front
        rings.append(
            (
                geom.add_vertex(outer_radius * c, -half, outer_radius * s + z_offset),
                geom.add_vertex(inner_radius * c, -half, inner_radius * s + z_offset),
                geom.add_vertex(outer_radius * c, half, outer_radius * s + z_offset),
                geom.add_vertex(inner_radius * c, half, inner_radius * s + z_offset),
            )
        )
    for i in range(segments):
        ob0, ib0, of0, inf0 = rings[i]
        ob1, ib1, of1, inf1 = rings[i + 1]
        # back face
        geom.add_face(ob0, ob1, ib1)
        geom.add_face(ob0, ib1, ib0)
        # front face
        geom.add_face(of0, inf1, of1)
        geom.add_face(of0, inf0, inf1)
        # outer curved wall
        geom.add_face(ob0, of1, ob1)
        geom.add_face(ob0, of0, of1)
        # inner curved wall
        geom.add_face(ib0, ib1, inf1)
        geom.add_face(ib0, inf1, inf0)

    # radial end caps
    ob0, ib0, of0, inf0 = rings[0]
    geom.add_face(ob0, ib0, inf0)
    geom.add_face(ob0, inf0, of0)
    obn, ibn, ofn, infn = rings[-1]
    geom.add_face(obn, infn, ibn)
    geom.add_face(obn, ofn, infn)
    return geom


def _tick_geometry(radius: float, length: float, thickness: float) -> MeshGeometry:
    # Tiny rectangular tick centered on the radial axis, with long dimension along local X.
    g = MeshGeometry()
    x0 = radius - length * 0.5
    x1 = radius + length * 0.5
    y0 = -thickness * 0.5
    y1 = thickness * 0.5
    z0 = -thickness * 0.5
    z1 = thickness * 0.5
    verts = [
        (x0, y0, z0),
        (x1, y0, z0),
        (x1, y1, z0),
        (x0, y1, z0),
        (x0, y0, z1),
        (x1, y0, z1),
        (x1, y1, z1),
        (x0, y1, z1),
    ]
    idx = [g.add_vertex(*v) for v in verts]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    for a, b, c in faces:
        g.add_face(idx[a], idx[b], idx[c])
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="manual_conduit_bender",
        meta={
            "category": "Electrical_Wiring",
            "small_class": "Conduit bender",
            "source_image": "picture/Electrical_Wiring/Conduit bender/001.png",
        },
    )

    safety_yellow = model.material("painted_safety_yellow", rgba=(0.96, 0.74, 0.05, 1.0))
    cast_black = model.material("black_cast_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    galvanized = model.material("galvanized_conduit", rgba=(0.64, 0.67, 0.68, 1.0))
    bright_mark = model.material("engraved_white_marking", rgba=(0.92, 0.92, 0.84, 1.0))
    warning_red = model.material("red_warning_label", rgba=(0.82, 0.10, 0.05, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = model.material("brass_fastener", rgba=(0.80, 0.56, 0.18, 1.0))

    pivot_z = 1.08

    # Root: welded yellow floor stand and pivot yoke.
    frame = model.part("stand_frame")
    frame.visual(
        Box((0.12, 0.25, 0.08)),
        origin=Origin(xyz=(-0.190, 0.0, pivot_z - 0.090)),
        material=safety_yellow,
        name="top_casting",
    )
    # Keep the fork ears outboard of the shoe face and the sample conduit.  The
    # generated version had these plates tucked into the channel face.
    frame.visual(
        Box((0.13, 0.026, 0.20)),
        origin=Origin(xyz=(0.0, -0.112, pivot_z)),
        material=safety_yellow,
        name="near_yoke_plate",
    )
    frame.visual(
        Box((0.13, 0.026, 0.20)),
        origin=Origin(xyz=(0.0, 0.112, pivot_z)),
        material=safety_yellow,
        name="far_yoke_plate",
    )
    # Outside pivot bosses/caps on the yoke show the visible axle support without intersecting the shoe.
    frame.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, -0.138, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="near_pivot_cap",
    )
    frame.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.0, 0.138, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="far_pivot_cap",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.278),
        origin=Origin(xyz=(0.0, 0.0, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_axle",
    )
    frame.visual(
        Box((0.24, 0.030, 0.085)),
        origin=Origin(xyz=(-0.075, -0.112, pivot_z - 0.055)),
        material=safety_yellow,
        name="near_yoke_saddle",
    )
    frame.visual(
        Box((0.24, 0.030, 0.085)),
        origin=Origin(xyz=(-0.075, 0.112, pivot_z - 0.055)),
        material=safety_yellow,
        name="far_yoke_saddle",
    )

    # Tripod-like stand tubes and feet, sized like a portable jobsite bender stand.
    top = (-0.10, 0.0, pivot_z - 0.06)
    front_top = (-0.14, -0.12, pivot_z - 0.09)
    rear_l = (-0.62, -0.46, 0.053)
    rear_r = (-0.62, 0.46, 0.053)
    front = (0.52, -0.50, 0.053)
    _add_tube(frame, top, rear_l, 0.024, safety_yellow, name="rear_left_leg")
    _add_tube(frame, top, rear_r, 0.024, safety_yellow, name="rear_right_leg")
    _add_tube(frame, front_top, front, 0.024, safety_yellow, name="front_leg")
    _add_tube(frame, rear_l, rear_r, 0.020, safety_yellow, name="rear_cross_tube")
    _add_tube(frame, rear_l, front, 0.016, safety_yellow, name="left_lower_brace")
    _add_tube(frame, rear_r, front, 0.016, safety_yellow, name="right_lower_brace")
    _add_tube(frame, (-0.35, -0.27, 0.52), (-0.06, -0.07, pivot_z - 0.07), 0.015, safety_yellow, name="left_upper_brace")
    _add_tube(frame, (-0.35, 0.27, 0.52), (-0.06, 0.07, pivot_z - 0.07), 0.015, safety_yellow, name="right_upper_brace")

    frame.visual(Box((0.26, 0.055, 0.050)), origin=Origin(xyz=(-0.62, -0.46, 0.028)), material=safety_yellow, name="rear_left_foot")
    frame.visual(Box((0.26, 0.055, 0.050)), origin=Origin(xyz=(-0.62, 0.46, 0.028)), material=safety_yellow, name="rear_right_foot")
    frame.visual(Box((0.26, 0.055, 0.050)), origin=Origin(xyz=(0.52, -0.50, 0.028), rpy=(0.0, 0.0, -0.15)), material=safety_yellow, name="front_foot")
    for x, y in [(-0.73, -0.46), (-0.51, -0.46), (-0.73, 0.46), (-0.51, 0.46), (0.41, -0.50), (0.63, -0.50)]:
        frame.visual(Box((0.035, 0.060, 0.018)), origin=Origin(xyz=(x, y, 0.009)), material=rubber_black, name="rubber_foot_pad")

    # A pressed/welded foot pedal with tread slots and support strut.
    frame.visual(
        Box((0.34, 0.12, 0.030)),
        origin=Origin(xyz=(-0.04, -0.33, 0.71), rpy=(0.0, 0.0, 0.12)),
        material=safety_yellow,
        name="foot_pedal_plate",
    )
    for i, dx in enumerate((-0.10, -0.035, 0.030, 0.095)):
        frame.visual(
            Box((0.016, 0.128, 0.012)),
            origin=Origin(xyz=(-0.04 + dx, -0.33, 0.729), rpy=(0.0, 0.0, 0.12)),
            material=dark_steel,
            name=f"pedal_tread_{i}",
        )
    _add_tube(frame, (-0.06, -0.28, 0.70), (-0.15, -0.07, pivot_z - 0.08), 0.013, safety_yellow, name="pedal_support_strut")

    # Brand/warning label on the upright tube, represented by real layered plates/stripes.
    frame.visual(Box((0.050, 0.006, 0.22)), origin=Origin(xyz=(-0.34, 0.235, 0.57), rpy=(0.48, 0.0, -0.50)), material=bright_mark, name="vertical_brand_label")
    for i, z in enumerate((0.50, 0.55, 0.60)):
        frame.visual(Box((0.054, 0.008, 0.013)), origin=Origin(xyz=(-0.34, 0.239, z), rpy=(0.48, 0.0, -0.50)), material=warning_red, name=f"label_stripe_{i}")

    frame.inertial = Inertial.from_geometry(Box((1.35, 1.02, 1.15)), mass=18.0, origin=Origin(xyz=(-0.08, 0.0, 0.55)))

    # Moving bending head: cast shoe, groove, hook lip, reinforced socket, handle.
    head = model.part("bending_head")
    shoe_mesh = mesh_from_geometry(_annular_sector_geometry(0.145, 0.315, 0.078, -112.0, 142.0), "cast_bender_shoe")
    head.visual(shoe_mesh, material=cast_black, name="curved_shoe")

    # Raised/rimmed conduit channel on the front face of the shoe.
    channel_mesh = mesh_from_geometry(_annular_sector_geometry(0.192, 0.238, 0.012, -102.0, 132.0), "shoe_channel_floor")
    head.visual(channel_mesh, origin=Origin(xyz=(0.0, 0.047, 0.0)), material=dark_steel, name="conduit_channel")
    rim_outer = mesh_from_geometry(_annular_sector_geometry(0.241, 0.262, 0.015, -102.0, 132.0), "outer_channel_rim")
    rim_inner = mesh_from_geometry(_annular_sector_geometry(0.168, 0.188, 0.015, -102.0, 132.0), "inner_channel_rim")
    head.visual(rim_outer, origin=Origin(xyz=(0.0, 0.051, 0.0)), material=cast_black, name="outer_channel_rim")
    head.visual(rim_inner, origin=Origin(xyz=(0.0, 0.051, 0.0)), material=cast_black, name="inner_channel_rim")

    # Hub and socket details.
    head.visual(
        Cylinder(radius=0.082, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_black,
        name="pivot_hub",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_bushing",
    )
    head.visual(
        Box((0.22, 0.082, 0.082)),
        origin=Origin(xyz=(0.405, 0.0, 0.0)),
        material=cast_black,
        name="reinforced_socket",
    )
    _add_tube(head, (0.500, 0.0, 0.0), (1.68, 0.0, 0.0), 0.025, dark_steel, name="long_handle_tube")
    head.visual(
        Cylinder(radius=0.035, length=0.18),
        origin=Origin(xyz=(1.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="rubber_handle_grip",
    )

    # Hook lip at the throat of the shoe.
    hook_pts = [
        (0.19 * math.cos(math.radians(-108)), 0.045, 0.19 * math.sin(math.radians(-108))),
        (0.25 * math.cos(math.radians(-118)), 0.047, 0.25 * math.sin(math.radians(-118))),
        (0.30 * math.cos(math.radians(-111)), 0.049, 0.30 * math.sin(math.radians(-111))),
    ]
    hook_geom = tube_from_spline_points(hook_pts, radius=0.010, samples_per_segment=16, radial_segments=18, up_hint=(0.0, 1.0, 0.0))
    head.visual(mesh_from_geometry(hook_geom, "conduit_hook_lip"), material=cast_black, name="hook_lip")

    # Cast ribs webbed from the hub to the outer shoe and socket.
    for i, ang in enumerate((-82.0, -35.0, 18.0, 68.0, 116.0)):
        r0 = 0.070
        r1 = 0.278
        a = (r0 * math.cos(math.radians(ang)), 0.037, r0 * math.sin(math.radians(ang)))
        b = (r1 * math.cos(math.radians(ang)), 0.037, r1 * math.sin(math.radians(ang)))
        _add_tube(head, a, b, 0.008, cast_black, name=f"cast_rib_{i}")
    _add_tube(head, (0.095, 0.037, 0.022), (0.500, 0.037, 0.030), 0.010, cast_black, name="socket_reinforcing_rib")
    _add_tube(head, (0.095, -0.037, -0.022), (0.500, -0.037, -0.030), 0.010, cast_black, name="rear_socket_rib")

    # Degree scale: individual raised tick meshes and numeric-looking bar groups.
    for idx, ang in enumerate(range(-90, 121, 15)):
        length = 0.034 if ang % 30 == 0 else 0.020
        tick = _tick_geometry(0.292, length, 0.006)
        tick.rotate((0.0, 1.0, 0.0), math.radians(-ang))
        tick.translate(0.0, 0.041, 0.0)
        head.visual(mesh_from_geometry(tick, f"degree_tick_{idx}"), material=bright_mark, name=f"degree_tick_{idx}")
    # three geometric "number" plaques beside major marks
    for i, ang in enumerate((-60.0, 0.0, 60.0)):
        label_r = 0.300
        x = label_r * math.cos(math.radians(ang))
        z = label_r * math.sin(math.radians(ang))
        head.visual(
            Box((0.030, 0.006, 0.012)),
            origin=Origin(xyz=(x, 0.041, z), rpy=(0.0, -math.radians(ang), 0.0)),
            material=bright_mark,
            name=f"degree_number_bar_{i}",
        )

    # Small brass/dark screws on the socket and shoe face.
    for i, (x, z) in enumerate(((0.335, 0.035), (0.335, -0.035), (0.475, 0.035), (0.475, -0.035))):
        head.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(xyz=(x, 0.044, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"socket_screw_{i}",
        )

    head.inertial = Inertial.from_geometry(Box((1.76, 0.16, 0.72)), mass=5.0, origin=Origin(xyz=(0.58, 0.0, 0.0)))

    # A sample galvanized conduit section sits in the shoe channel and continues outward.
    conduit = model.part("sample_conduit")
    conduit_arc = tube_from_spline_points(
        _arc_points(0.215, -97.0, 67.0, y=0.072, samples=44),
        radius=0.014,
        samples_per_segment=3,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    conduit_arc.merge(
        tube_from_spline_points(
            [
                (0.215 * math.cos(math.radians(67.0)), 0.072, 0.215 * math.sin(math.radians(67.0))),
                (0.38, 0.072, 0.40),
                (0.68, 0.072, 0.48),
            ],
            radius=0.014,
            samples_per_segment=12,
            radial_segments=20,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        )
    )
    conduit.visual(mesh_from_geometry(conduit_arc, "galvanized_conduit_segment"), material=galvanized, name="seated_conduit")
    conduit.visual(
        Box((0.078, 0.012, 0.024)),
        origin=Origin(xyz=(0.41, 0.086, 0.39), rpy=(0.0, -0.70, 0.0)),
        material=bright_mark,
        name="conduit_stamp",
    )
    conduit.inertial = Inertial.from_geometry(Box((0.92, 0.050, 0.74)), mass=0.9, origin=Origin(xyz=(0.20, 0.070, 0.02)))

    model.articulation(
        "head_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        # The handle extends along local +X. Positive rotation about +Y lowers it like a real bender stroke.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.75, lower=0.0, upper=1.05),
    )
    model.articulation(
        "conduit_seat",
        ArticulationType.FIXED,
        parent=head,
        child=conduit,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("stand_frame")
    head = object_model.get_part("bending_head")
    conduit = object_model.get_part("sample_conduit")
    joint = object_model.get_articulation("head_pivot")

    ctx.check(
        "asset is small class Conduit bender",
        object_model.meta.get("small_class") == "Conduit bender"
        and object_model.meta.get("category") == "Electrical_Wiring",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "visible bender subassemblies exist",
        all(
            name in [v.name for v in head.visuals]
            for name in (
                "curved_shoe",
                "conduit_channel",
                "hook_lip",
                "long_handle_tube",
                "reinforced_socket",
                "degree_tick_0",
            )
        )
        and frame.get_visual("foot_pedal_plate") is not None
        and conduit.get_visual("seated_conduit") is not None,
        details="Missing shoe, channel, hook, handle, foot pedal, degree marks, or sample conduit.",
    )
    ctx.check(
        "primary bending joint has realistic limits",
        joint.articulation_type == ArticulationType.REVOLUTE
        and joint.axis == (0.0, 1.0, 0.0)
        and joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and 0.9 <= joint.motion_limits.upper <= 1.2,
        details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
    )

    ctx.allow_overlap(
        frame,
        head,
        elem_a="pivot_axle",
        elem_b="pivot_bushing",
        reason="The stand axle intentionally passes through the bending head bushing as the revolute bearing.",
    )
    ctx.allow_overlap(
        head,
        frame,
        elem_a="pivot_hub",
        elem_b="pivot_axle",
        reason="The black cast pivot hub wraps around the stand axle as a captured bearing sleeve.",
    )
    ctx.expect_contact(
        frame,
        head,
        elem_a="pivot_axle",
        elem_b="pivot_bushing",
        name="pivot axle captures the bending head bushing",
    )
    ctx.expect_contact(
        head,
        frame,
        elem_a="pivot_hub",
        elem_b="pivot_axle",
        name="pivot hub rides on the stand axle",
    )

    # The conduit is intentionally seated just proud of the channel; it should be centered over the
    # curved shoe footprint without being a floating unrelated rod.
    ctx.expect_overlap(
        conduit,
        head,
        axes="xz",
        min_overlap=0.18,
        elem_a="seated_conduit",
        elem_b="conduit_channel",
        name="sample conduit follows the shoe channel",
    )
    ctx.expect_gap(
        conduit,
        head,
        axis="y",
        max_gap=0.035,
        max_penetration=0.001,
        positive_elem="seated_conduit",
        negative_elem="conduit_channel",
        name="sample conduit sits just proud of the channel face",
    )

    rest_aabb = ctx.part_element_world_aabb(head, elem="long_handle_tube")
    with ctx.pose({joint: 0.85}):
        bent_aabb = ctx.part_element_world_aabb(head, elem="long_handle_tube")
        ctx.expect_overlap(
            conduit,
            head,
            axes="xz",
            min_overlap=0.12,
            elem_a="seated_conduit",
            elem_b="conduit_channel",
            name="conduit remains seated during bending stroke",
        )

    ctx.check(
        "handle lowers through bending stroke",
        rest_aabb is not None
        and bent_aabb is not None
        and bent_aabb[0][2] < rest_aabb[0][2] - 0.35,
        details=f"rest_handle_aabb={rest_aabb}, bent_handle_aabb={bent_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
