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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PIVOT_Z = 1.05
LOWER_LIMIT = math.radians(-27.5)
UPPER_LIMIT = math.radians(40.0)


def _cylinder_between(part, name, p0, p1, radius, material, *, segments=24):
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError(f"zero-length cylinder {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _annular_sector(
    *,
    inner_radius: float,
    outer_radius: float,
    thickness_y: float,
    theta0: float,
    theta1: float,
    segments: int = 48,
    y_offset: float = 0.0,
) -> MeshGeometry:
    """Extruded annular sector in the local XZ plane, with thickness along Y."""
    geom = MeshGeometry()
    front = y_offset + thickness_y * 0.5
    back = y_offset - thickness_y * 0.5
    angles = [theta0 + (theta1 - theta0) * i / segments for i in range(segments + 1)]
    # Per station: outer/front, inner/front, outer/back, inner/back.
    for theta in angles:
        c = math.cos(theta)
        s = math.sin(theta)
        geom.add_vertex(outer_radius * c, front, outer_radius * s)
        geom.add_vertex(inner_radius * c, front, inner_radius * s)
        geom.add_vertex(outer_radius * c, back, outer_radius * s)
        geom.add_vertex(inner_radius * c, back, inner_radius * s)
    for i in range(segments):
        a = 4 * i
        b = 4 * (i + 1)
        # Front face.
        geom.add_face(a, b, b + 1)
        geom.add_face(a, b + 1, a + 1)
        # Back face.
        geom.add_face(a + 2, b + 3, b + 2)
        geom.add_face(a + 2, a + 3, b + 3)
        # Outer radius wall.
        geom.add_face(a, a + 2, b + 2)
        geom.add_face(a, b + 2, b)
        # Inner radius wall.
        geom.add_face(a + 1, b + 1, b + 3)
        geom.add_face(a + 1, b + 3, a + 3)
    # Radial end caps.
    a = 0
    geom.add_face(a, a + 1, a + 3)
    geom.add_face(a, a + 3, a + 2)
    a = 4 * segments
    geom.add_face(a, a + 3, a + 1)
    geom.add_face(a, a + 2, a + 3)
    return geom


def _tube_along_arc(
    *,
    radius_path: float,
    tube_radius: float,
    theta0: float,
    theta1: float,
    segments: int = 64,
    radial_segments: int = 18,
    y_offset: float = 0.0,
    cap_ends: bool = True,
) -> MeshGeometry:
    """Circular tube following an arc around the pivot in the local XZ plane."""
    geom = MeshGeometry()
    angles = [theta0 + (theta1 - theta0) * i / segments for i in range(segments + 1)]
    for theta in angles:
        c = math.cos(theta)
        s = math.sin(theta)
        cx, cy, cz = radius_path * c, y_offset, radius_path * s
        # Outward radial and width vectors define the circular tube section.
        radial = (c, 0.0, s)
        width = (0.0, 1.0, 0.0)
        for j in range(radial_segments):
            phi = 2.0 * math.pi * j / radial_segments
            cp = math.cos(phi)
            sp = math.sin(phi)
            geom.add_vertex(
                cx + tube_radius * (cp * radial[0] + sp * width[0]),
                cy + tube_radius * (cp * radial[1] + sp * width[1]),
                cz + tube_radius * (cp * radial[2] + sp * width[2]),
            )
    for i in range(segments):
        a = i * radial_segments
        b = (i + 1) * radial_segments
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            geom.add_face(a + j, b + j, b + j2)
            geom.add_face(a + j, b + j2, a + j2)
    if cap_ends:
        start_center = geom.add_vertex(
            radius_path * math.cos(theta0), y_offset, radius_path * math.sin(theta0)
        )
        end_center = geom.add_vertex(
            radius_path * math.cos(theta1), y_offset, radius_path * math.sin(theta1)
        )
        start = 0
        end = segments * radial_segments
        for j in range(radial_segments):
            j2 = (j + 1) % radial_segments
            geom.add_face(start_center, start + j2, start + j)
            geom.add_face(end_center, end + j, end + j2)
    return geom


def _add_arc_tube_visual(
    part,
    name: str,
    *,
    radius_path: float,
    tube_radius: float,
    theta0: float,
    theta1: float,
    material,
    y_offset: float = 0.0,
    segments: int = 64,
    radial_segments: int = 18,
):
    geom = _tube_along_arc(
        radius_path=radius_path,
        tube_radius=tube_radius,
        theta0=theta0,
        theta1=theta1,
        segments=segments,
        radial_segments=radial_segments,
        y_offset=y_offset,
    )
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def _add_radial_box(part, name, radius, theta, size, material, *, y=0.0):
    part.visual(
        Box(size),
        origin=Origin(
            xyz=(radius * math.cos(theta), y, radius * math.sin(theta)),
            rpy=(0.0, -theta, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_conduit_bender",
        meta={"small_class": "Conduit bender", "domain": "Electrical_Wiring"},
    )

    yellow = Material("painted_yellow_steel", rgba=(1.0, 0.72, 0.02, 1.0))
    black = Material("black_cast_iron", rgba=(0.015, 0.015, 0.018, 1.0))
    dark = Material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("black_rubber_grip", rgba=(0.005, 0.006, 0.007, 1.0))
    galvanized = Material("galvanized_conduit", rgba=(0.70, 0.74, 0.75, 1.0))
    brass = Material("brass_terminal_screw", rgba=(0.86, 0.64, 0.25, 1.0))
    white = Material("white_marking_paint", rgba=(0.96, 0.95, 0.88, 1.0))
    label = Material("warning_label", rgba=(1.0, 0.92, 0.10, 1.0))

    frame = model.part("frame")

    # Floor rails and splayed tripod/A-frame legs.
    _cylinder_between(frame, "front_foot", (-0.62, -0.46, 0.035), (-0.62, 0.46, 0.035), 0.020, yellow)
    _cylinder_between(frame, "rear_foot", (0.54, -0.50, 0.035), (0.54, 0.50, 0.035), 0.020, yellow)
    _cylinder_between(frame, "front_leg_0", (-0.62, -0.38, 0.055), (-0.055, -0.060, 0.925), 0.018, yellow)
    _cylinder_between(frame, "front_leg_1", (-0.62, 0.38, 0.055), (-0.055, 0.060, 0.925), 0.018, yellow)
    _cylinder_between(frame, "rear_leg_0", (0.54, -0.42, 0.055), (-0.015, -0.065, 0.925), 0.018, yellow)
    _cylinder_between(frame, "rear_leg_1", (0.54, 0.42, 0.055), (-0.015, 0.065, 0.925), 0.018, yellow)
    _cylinder_between(frame, "lower_cross_brace", (-0.513, -0.319, 0.22), (0.435, 0.353, 0.22), 0.010, dark)
    _cylinder_between(frame, "side_cross_brace", (-0.526, 0.327, 0.20), (0.448, -0.361, 0.20), 0.010, dark)

    frame.visual(Box((0.18, 0.20, 0.075)), origin=Origin(xyz=(-0.050, 0.0, 0.760)), material=yellow, name="top_socket")
    frame.visual(Box((0.12, 0.16, 0.070)), origin=Origin(xyz=(-0.065, 0.0, 0.825)), material=yellow, name="reinforced_socket")
    frame.visual(Box((0.025, 0.16, 0.18)), origin=Origin(xyz=(-0.135, 0.0, 0.950)), material=yellow, name="socket_neck")

    # Dark yoke plates straddle the moving shoe with real side clearance.
    frame.visual(Box((0.18, 0.012, 0.20)), origin=Origin(xyz=(0.0, -0.064, PIVOT_Z - 0.010)), material=dark, name="yoke_plate_0")
    frame.visual(Box((0.18, 0.012, 0.20)), origin=Origin(xyz=(0.0, 0.064, PIVOT_Z - 0.010)), material=dark, name="yoke_plate_1")
    frame.visual(Cylinder(radius=0.055, length=0.008), origin=Origin(xyz=(0.0, -0.054, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="pivot_bushing_0")
    frame.visual(Cylinder(radius=0.055, length=0.008), origin=Origin(xyz=(0.0, 0.054, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="pivot_bushing_1")
    frame.visual(Cylinder(radius=0.010, length=0.030), origin=Origin(xyz=(0.075, -0.071, PIVOT_Z + 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="terminal_screw_0")
    frame.visual(Cylinder(radius=0.010, length=0.030), origin=Origin(xyz=(0.075, 0.071, PIVOT_Z + 0.075), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="terminal_screw_1")

    # Electrical warning and model labels are physical raised plates, not just color.
    frame.visual(Box((0.050, 0.004, 0.120)), origin=Origin(xyz=(-0.050, 0.102, 0.760)), material=label, name="warning_label")
    for i in range(3):
        frame.visual(Box((0.040, 0.005, 0.006)), origin=Origin(xyz=(-0.050, 0.1065, 0.725 + i * 0.030)), material=black, name=f"label_stripe_{i}")

    head = model.part("bender_head")
    theta0 = math.radians(-68.0)
    theta1 = math.radians(146.0)

    # Cast shoe: web, raised side lips, and a clear U-groove for conduit.
    head.visual(
        mesh_from_geometry(
            _annular_sector(inner_radius=0.058, outer_radius=0.166, thickness_y=0.048, theta0=theta0, theta1=theta1, segments=72),
            "shoe_web",
        ),
        material=black,
        name="shoe_web",
    )
    head.visual(
        mesh_from_geometry(
            _annular_sector(inner_radius=0.164, outer_radius=0.178, thickness_y=0.030, theta0=theta0, theta1=theta1, segments=72),
            "channel_bottom",
        ),
        material=dark,
        name="channel_bottom",
    )
    head.visual(
        mesh_from_geometry(
            _annular_sector(inner_radius=0.160, outer_radius=0.207, thickness_y=0.010, theta0=theta0, theta1=theta1, segments=72, y_offset=-0.027),
            "channel_lip_0",
        ),
        material=black,
        name="channel_lip_0",
    )
    head.visual(
        mesh_from_geometry(
            _annular_sector(inner_radius=0.160, outer_radius=0.207, thickness_y=0.010, theta0=theta0, theta1=theta1, segments=72, y_offset=0.027),
            "channel_lip_1",
        ),
        material=black,
        name="channel_lip_1",
    )
    head.visual(Cylinder(radius=0.062, length=0.090), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=black, name="pivot_hub")
    head.visual(Cylinder(radius=0.034, length=0.118), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark, name="axle_pin")

    # Cast radial ribs and web bosses.
    for idx, ang in enumerate((math.radians(-38), math.radians(18), math.radians(78), math.radians(128))):
        _add_radial_box(head, f"cast_rib_{idx}", 0.108, ang, (0.014, 0.042, 0.125), black)
    head.visual(Box((0.090, 0.070, 0.035)), origin=Origin(xyz=(0.160, 0.075, -0.020), rpy=(0.0, -0.12, 0.0)), material=black, name="handle_socket")
    head.visual(Box((0.090, 0.052, 0.055)), origin=Origin(xyz=(0.115, 0.035, -0.020), rpy=(0.0, -0.10, 0.0)), material=black, name="handle_root_web")
    _cylinder_between(head, "handle_tube", (0.172, 0.095, -0.030), (0.880, 0.095, -0.180), 0.014, dark)
    _cylinder_between(head, "rubber_grip", (0.690, 0.095, -0.140), (0.910, 0.095, -0.187), 0.019, rubber)

    # Foot pedal and hook lip mounted to the shoe casting.
    head.visual(Box((0.160, 0.050, 0.018)), origin=Origin(xyz=(0.230, -0.145, -0.100), rpy=(0.0, -0.28, 0.0)), material=yellow, name="foot_pedal")
    head.visual(Box((0.055, 0.050, 0.075)), origin=Origin(xyz=(0.176, -0.095, -0.090), rpy=(0.0, -0.18, 0.0)), material=yellow, name="pedal_web")
    head.visual(Box((0.060, 0.092, 0.075)), origin=Origin(xyz=(0.155, -0.055, -0.085), rpy=(0.0, -0.18, 0.0)), material=yellow, name="pedal_root_web")
    _add_arc_tube_visual(
        head,
        "hook_lip",
        radius_path=0.224,
        tube_radius=0.008,
        theta0=math.radians(-82),
        theta1=math.radians(-66),
        material=black,
        y_offset=-0.044,
        segments=14,
        radial_segments=14,
    )
    head.visual(Box((0.040, 0.026, 0.040)), origin=Origin(xyz=(0.064, -0.034, -0.186), rpy=(0.0, -0.30, 0.0)), material=black, name="hook_web")
    _add_radial_box(
        head,
        "conduit_retainer",
        0.190,
        math.radians(25),
        (0.032, 0.0175, 0.024),
        dark,
        y=0.01875,
    )

    # White degree ticks and raised number pads along the shoe face.
    for idx, deg in enumerate((-45, -15, 15, 45, 75, 105, 135)):
        length = 0.034 if idx % 2 == 0 else 0.022
        _add_radial_box(head, f"degree_tick_{idx}", 0.187, math.radians(deg), (0.004, 0.004, length), white, y=-0.034)
    for idx, deg in enumerate((0, 45, 90)):
        _add_radial_box(head, f"degree_pad_{idx}", 0.128, math.radians(deg), (0.030, 0.004, 0.018), white, y=-0.026)

    conduit = model.part("conduit")
    _add_arc_tube_visual(
        conduit,
        "conduit_arc",
        radius_path=0.190,
        tube_radius=0.010,
        theta0=math.radians(-56),
        theta1=math.radians(128),
        material=galvanized,
        y_offset=0.0,
        segments=72,
        radial_segments=20,
    )
    # Short straight stubs make the sample read as a real EMT segment in the groove.
    _cylinder_between(
        conduit,
        "conduit_straight_tail",
        (0.190 * math.cos(math.radians(-56)), 0.0, 0.190 * math.sin(math.radians(-56))),
        (0.300, 0.0, -0.205),
        0.010,
        galvanized,
    )
    _cylinder_between(
        conduit,
        "conduit_exit_stub",
        (0.190 * math.cos(math.radians(128)), 0.0, 0.190 * math.sin(math.radians(128))),
        (-0.205, 0.0, 0.235),
        0.010,
        galvanized,
    )

    model.articulation(
        "frame_to_bender_head",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.2, lower=LOWER_LIMIT, upper=UPPER_LIMIT),
        meta={"mechanism": "manual handle and shoe rotate about the yoke axle"},
    )
    model.articulation(
        "head_to_conduit",
        ArticulationType.FIXED,
        parent=head,
        child=conduit,
        origin=Origin(),
        meta={"fit": "sample EMT conduit seated proud in the shoe groove"},
    )

    return model


def _aabb_gap_z(ctx: TestContext, part, elem: str, floor: float):
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    return aabb[0][2] - floor


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("bender_head")
    conduit = object_model.get_part("conduit")
    joint = object_model.get_articulation("frame_to_bender_head")
    fixed = object_model.get_articulation("head_to_conduit")

    ctx.check(
        "small class is conduit bender",
        object_model.name == "electrical_conduit_bender"
        and object_model.meta.get("small_class") == "Conduit bender",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "primary revolute mechanism has realistic limits",
        joint.articulation_type == ArticulationType.REVOLUTE
        and joint.motion_limits is not None
        and math.isclose(joint.motion_limits.lower, LOWER_LIMIT, abs_tol=1e-6)
        and math.isclose(joint.motion_limits.upper, UPPER_LIMIT, abs_tol=1e-6),
        details=f"type={joint.articulation_type}, limits={joint.motion_limits}",
    )
    ctx.check(
        "sample conduit is fixed to moving shoe for seated bending sample",
        fixed.articulation_type == ArticulationType.FIXED,
        details=f"conduit joint type={fixed.articulation_type}",
    )

    frame_names = {v.name for v in frame.visuals}
    head_names = {v.name for v in head.visuals}
    conduit_names = {v.name for v in conduit.visuals}
    ctx.check(
        "frame has tripod feet yoke socket bushings labels and hardware",
        {"front_foot", "rear_foot", "front_leg_0", "rear_leg_0", "top_socket", "reinforced_socket", "yoke_plate_0", "yoke_plate_1", "pivot_bushing_0", "warning_label", "terminal_screw_0"}.issubset(frame_names),
        details=str(sorted(frame_names)),
    )
    ctx.check(
        "head has curved shoe groove handle foot pedal hook ribs and markings",
        {"shoe_web", "channel_bottom", "channel_lip_0", "channel_lip_1", "pivot_hub", "handle_tube", "rubber_grip", "foot_pedal", "hook_lip", "conduit_retainer", "cast_rib_0", "degree_tick_0", "degree_pad_1"}.issubset(head_names),
        details=str(sorted(head_names)),
    )
    ctx.check(
        "galvanized sample conduit has curved seated arc and straight stubs",
        {"conduit_arc", "conduit_straight_tail", "conduit_exit_stub"}.issubset(conduit_names),
        details=str(sorted(conduit_names)),
    )

    ctx.allow_overlap(
        head,
        frame,
        elem_a="axle_pin",
        elem_b="pivot_bushing_0",
        reason="The rotating axle is intentionally captured inside the fixed bushing bore as a bearing fit.",
    )
    ctx.allow_overlap(
        head,
        frame,
        elem_a="axle_pin",
        elem_b="pivot_bushing_1",
        reason="The rotating axle is intentionally captured inside the opposite fixed bushing bore as a bearing fit.",
    )

    # Seating and bearing checks at rest: conduit follows the channel without being buried;
    # the hub remains between, but not through, the yoke bushings.
    ctx.expect_overlap(conduit, head, axes="xz", elem_a="conduit_arc", elem_b="channel_bottom", min_overlap=0.15, name="conduit arc follows curved shoe channel")
    ctx.expect_gap(conduit, head, axis="y", positive_elem="conduit_arc", negative_elem="channel_lip_0", min_gap=0.001, name="conduit clears negative channel lip")
    ctx.expect_gap(head, conduit, axis="y", positive_elem="channel_lip_1", negative_elem="conduit_arc", min_gap=0.001, name="conduit clears positive channel lip")
    ctx.expect_gap(frame, head, axis="y", positive_elem="pivot_bushing_1", negative_elem="pivot_hub", min_gap=0.002, name="positive bushing clears hub face")
    ctx.expect_gap(head, frame, axis="y", positive_elem="pivot_hub", negative_elem="pivot_bushing_0", min_gap=0.002, name="negative bushing clears hub face")
    ctx.expect_overlap(head, frame, axes="xz", elem_a="axle_pin", elem_b="pivot_bushing_0", min_overlap=0.030, name="axle centered in negative bushing bore")
    ctx.expect_overlap(head, frame, axes="xz", elem_a="axle_pin", elem_b="pivot_bushing_1", min_overlap=0.030, name="axle centered in positive bushing bore")
    ctx.expect_gap(head, conduit, axis="y", positive_elem="conduit_retainer", negative_elem="conduit_arc", max_gap=0.001, max_penetration=0.0002, name="retainer just touches seated conduit")

    rest_grip = ctx.part_element_world_aabb(head, elem="rubber_grip")
    poses = {
        "lower_limit": LOWER_LIMIT + 0.01,
        "rest": 0.0,
        "mid_stroke": (LOWER_LIMIT + UPPER_LIMIT) * 0.5,
        "upper_limit": UPPER_LIMIT - 0.01,
    }
    for pose_name, q in poses.items():
        with ctx.pose({joint: q}):
            ctx.expect_gap(frame, head, axis="y", positive_elem="pivot_bushing_1", negative_elem="pivot_hub", min_gap=0.002, name=f"{pose_name} positive axle side clearance")
            ctx.expect_gap(head, frame, axis="y", positive_elem="pivot_hub", negative_elem="pivot_bushing_0", min_gap=0.002, name=f"{pose_name} negative axle side clearance")
            ctx.expect_gap(conduit, head, axis="y", positive_elem="conduit_arc", negative_elem="channel_lip_0", min_gap=0.001, name=f"{pose_name} conduit side clearance 0")
            ctx.expect_gap(head, conduit, axis="y", positive_elem="channel_lip_1", negative_elem="conduit_arc", min_gap=0.001, name=f"{pose_name} conduit side clearance 1")
            grip_clearance = _aabb_gap_z(ctx, head, "rubber_grip", 0.035)
            head_clearance = _aabb_gap_z(ctx, head, "foot_pedal", 0.035)
            ctx.check(
                f"{pose_name} moving handle and pedal clear floor rails",
                grip_clearance is not None
                and head_clearance is not None
                and grip_clearance > 0.18
                and head_clearance > 0.18,
                details=f"grip_clearance={grip_clearance}, pedal_clearance={head_clearance}",
            )
            ctx.expect_gap(head, frame, axis="z", positive_elem="pivot_hub", negative_elem="top_socket", min_gap=0.030, name=f"{pose_name} hub clears fixed socket")

    with ctx.pose({joint: UPPER_LIMIT - 0.01}):
        upper_grip = ctx.part_element_world_aabb(head, elem="rubber_grip")
    ctx.check(
        "positive handle stroke visibly raises rubber grip",
        rest_grip is not None and upper_grip is not None and upper_grip[0][2] > rest_grip[0][2] + 0.35,
        details=f"rest={rest_grip}, upper={upper_grip}",
    )

    return ctx.report()


object_model = build_object_model()
