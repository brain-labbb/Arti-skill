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
    tube_from_spline_points,
)


BLUE = Material("blue_painted_cast_metal", rgba=(0.00, 0.46, 0.78, 1.0))
DARK_BLUE = Material("dark_recessed_marking", rgba=(0.02, 0.07, 0.11, 1.0))
STEEL = Material("brushed_galvanized_steel", rgba=(0.72, 0.73, 0.68, 1.0))
DARK_STEEL = Material("dark_oxide_screw_heads", rgba=(0.05, 0.05, 0.045, 1.0))
COPPER = Material("warm_copper_conduit", rgba=(0.78, 0.36, 0.16, 1.0))
RUBBER = Material("black_rubber_grip", rgba=(0.01, 0.012, 0.012, 1.0))
WHITE = Material("white_cast_degree_paint", rgba=(0.92, 0.90, 0.82, 1.0))


def _annular_sector_mesh(
    inner_radius: float,
    outer_radius: float,
    width_y: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 64,
) -> MeshGeometry:
    """Extruded annular sector in the XZ plane, centered on the Y axis."""

    mesh = MeshGeometry()
    angles = [
        start_angle + (end_angle - start_angle) * i / segments
        for i in range(segments + 1)
    ]
    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        for y in (-width_y / 2.0, width_y / 2.0):
            mesh.add_vertex(outer_radius * ca, y, outer_radius * sa)
            mesh.add_vertex(inner_radius * ca, y, inner_radius * sa)

    for i in range(segments):
        a = 4 * i
        b = 4 * (i + 1)
        # Front face (negative Y)
        mesh.add_face(a, b, b + 1)
        mesh.add_face(a, b + 1, a + 1)
        # Back face (positive Y)
        mesh.add_face(a + 2, b + 3, b + 2)
        mesh.add_face(a + 2, a + 3, b + 3)
        # Outer cylindrical wall
        mesh.add_face(a, a + 2, b + 2)
        mesh.add_face(a, b + 2, b)
        # Inner cylindrical wall
        mesh.add_face(a + 1, b + 1, b + 3)
        mesh.add_face(a + 1, b + 3, a + 3)

    # Flat end caps.
    for a in (0, 4 * segments):
        mesh.add_face(a, a + 1, a + 3)
        mesh.add_face(a, a + 3, a + 2)
    return mesh


def _add_radial_bar(
    part,
    *,
    name: str,
    radius: float,
    angle: float,
    length: float,
    thickness_z: float,
    width_y: float,
    material,
    radial_offset: float = 0.0,
) -> None:
    """Add a small rectangular bar whose local X axis points radially."""

    r = radius + radial_offset
    part.visual(
        Box((length, width_y, thickness_z)),
        origin=Origin(
            xyz=(r * math.cos(angle), -0.034, r * math.sin(angle)),
            rpy=(0.0, -angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _tube_mesh(points, *, radius: float, name: str, radial_segments: int = 20):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            radial_segments=radial_segments,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_conduit_bender")

    shoe = model.part("shoe_head")

    shoe_body = _annular_sector_mesh(
        0.066,
        0.158,
        0.055,
        math.radians(-118.0),
        math.radians(166.0),
        segments=80,
    )
    shoe.visual(
        mesh_from_geometry(shoe_body, "curved_shoe_mesh"),
        material=BLUE,
        name="curved_shoe",
    )
    shoe.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLUE,
        name="front_hub_web",
    )

    # White/silver cast-in degree scale on the front face.
    degree_plate = _annular_sector_mesh(
        0.120,
        0.154,
        0.006,
        math.radians(-104.0),
        math.radians(148.0),
        segments=54,
    )
    shoe.visual(
        mesh_from_geometry(degree_plate, "degree_scale_mesh"),
        origin=Origin(xyz=(0.0, -0.032, 0.0)),
        material=WHITE,
        name="degree_scale",
    )

    # Cast radial ribs tying the hub to the shoe rim.
    for i, angle_deg in enumerate((-88, -52, -18, 24, 68, 112)):
        angle = math.radians(angle_deg)
        r_mid = 0.085
        shoe.visual(
            Box((0.098, 0.040, 0.014)),
            origin=Origin(
                xyz=(r_mid * math.cos(angle), 0.0, r_mid * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=BLUE,
            name=f"cast_rib_{i}",
        )

    # Front pivot cap and a steel pin that captures the moving bushing.
    shoe.visual(
        Cylinder(radius=0.037, length=0.014),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="pivot_cap",
    )
    shoe.visual(
        Cylinder(radius=0.012, length=0.130),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pivot_pin",
    )

    # Foot pedal and serrated galvanized tread for boot pressure.
    shoe.visual(
        Box((0.175, 0.086, 0.028)),
        origin=Origin(xyz=(-0.125, 0.0, -0.155), rpy=(0.0, math.radians(8.0), 0.0)),
        material=BLUE,
        name="foot_pedal",
    )
    shoe.visual(
        Box((0.150, 0.072, 0.006)),
        origin=Origin(xyz=(-0.145, -0.002, -0.137), rpy=(0.0, math.radians(8.0), 0.0)),
        material=STEEL,
        name="serrated_tread_plate",
    )
    for i, x in enumerate((-0.175, -0.153, -0.131, -0.109, -0.087)):
        shoe.visual(
            Box((0.006, 0.078, 0.012)),
            origin=Origin(xyz=(x, -0.006, -0.137), rpy=(0.0, math.radians(8.0), 0.0)),
            material=DARK_STEEL,
            name=f"tread_tooth_{i}",
        )

    # Hook lip at the shoe entrance that traps the conduit against the groove.
    hook_angle = math.radians(157.0)
    shoe.visual(
        Box((0.058, 0.070, 0.030)),
        origin=Origin(
            xyz=(0.160 * math.cos(hook_angle), -0.001, 0.160 * math.sin(hook_angle)),
            rpy=(0.0, -hook_angle + math.pi / 2.0, 0.0),
        ),
        material=BLUE,
        name="hook_lip",
    )
    shoe.visual(
        Box((0.044, 0.055, 0.012)),
        origin=Origin(
            xyz=(0.177 * math.cos(hook_angle), -0.038, 0.177 * math.sin(hook_angle)),
            rpy=(0.0, -hook_angle + math.pi / 2.0, 0.0),
        ),
        material=STEEL,
        name="hook_wear_face",
    )

    # Tangible degree tick marks, distributed symmetrically across the cast scale.
    scale_center_deg = 22.0
    tick_spacing_deg = 23.0
    tick_count = 11
    tick_mid = tick_count // 2
    for i in range(tick_count):
        angle_deg = scale_center_deg + (i - tick_mid) * tick_spacing_deg
        _add_radial_bar(
            shoe,
            name=f"degree_tick_{i}",
            radius=0.137,
            angle=math.radians(angle_deg),
            length=0.023 if i in (0, tick_mid, tick_count - 1) else 0.015,
            thickness_z=0.0035,
            width_y=0.004,
            material=DARK_BLUE,
        )

    # Separate fixed sample conduit: copper-colored, continuous, and seated over
    # the front channel/groove rather than painted onto the shoe.
    conduit = model.part("sample_conduit")
    arc_points = []
    for angle_deg in (-108, -86, -64, -42, -20, 2, 24, 38):
        angle = math.radians(angle_deg)
        arc_points.append((0.149 * math.cos(angle), -0.046, 0.149 * math.sin(angle)))
    conduit_path = [
        (-0.045, -0.046, -0.360),
        (-0.045, -0.046, -0.220),
        *arc_points,
        (0.245, -0.046, 0.107),
        (0.425, -0.046, 0.128),
    ]
    conduit.visual(
        _tube_mesh(conduit_path, radius=0.006, name="sample_conduit_tube", radial_segments=24),
        material=COPPER,
        name="conduit_tube",
    )

    model.articulation(
        "shoe_to_conduit",
        ArticulationType.FIXED,
        parent=shoe,
        child=conduit,
        origin=Origin(),
    )

    # Moving handle/pressure arm.  Its frame is exactly on the shoe bend axis.
    handle = model.part("swing_handle")
    handle.visual(
        Cylinder(radius=0.027, length=0.037),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLUE,
        name="pivot_bushing",
    )
    handle.visual(
        Box((0.090, 0.026, 0.045)),
        origin=Origin(xyz=(0.054, 0.043, 0.023), rpy=(0.0, math.radians(-23.0), 0.0)),
        material=BLUE,
        name="yoke_cheek",
    )
    handle.visual(
        Box((0.125, 0.052, 0.050)),
        origin=Origin(xyz=(0.113, 0.069, 0.052), rpy=(0.0, math.radians(-15.0), 0.0)),
        material=BLUE,
        name="handle_socket",
    )
    handle_points = [
        (0.078, 0.073, 0.060),
        (0.210, 0.073, 0.090),
        (0.500, 0.073, 0.154),
        (0.775, 0.073, 0.215),
    ]
    handle.visual(
        _tube_mesh(handle_points, radius=0.015, name="handle_tube_mesh", radial_segments=22),
        material=BLUE,
        name="handle_tube",
    )
    handle.visual(
        _tube_mesh(
            [
                (0.610, 0.073, 0.179),
                (0.705, 0.073, 0.200),
                (0.815, 0.073, 0.224),
            ],
            radius=0.018,
            name="rubber_grip_mesh",
            radial_segments=22,
        ),
        material=RUBBER,
        name="rubber_grip",
    )
    for i, x in enumerate((0.640, 0.672, 0.704, 0.736, 0.768)):
        handle.visual(
            Cylinder(radius=0.020, length=0.005),
            origin=Origin(
                xyz=(x, 0.073, 0.185 + (x - 0.640) * 0.22),
                rpy=(0.0, math.pi / 2.0 - math.radians(12.0), 0.0),
            ),
            material=DARK_BLUE,
            name=f"grip_rib_{i}",
        )
    for i, offset in enumerate((0.0, 0.018)):
        handle.visual(
            Box((0.370, 0.007, 0.006)),
            origin=Origin(
                xyz=(0.380, 0.073 + offset - 0.009, 0.135 + offset * 0.18),
                rpy=(0.0, math.radians(-12.0), 0.0),
            ),
            material=BLUE,
            name=f"raised_handle_rib_{i}",
        )
    handle.visual(
        Cylinder(radius=0.017, length=0.042),
        origin=Origin(xyz=(0.250, 0.050, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pressure_roller",
    )
    handle.visual(
        Box((0.100, 0.065, 0.035)),
        origin=Origin(xyz=(0.200, 0.060, 0.095), rpy=(0.0, math.radians(-10.0), 0.0)),
        material=BLUE,
        name="roller_bracket",
    )

    model.articulation(
        "shoe_to_swing_handle",
        ArticulationType.REVOLUTE,
        parent=shoe,
        child=handle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.4, lower=0.0, upper=1.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoe = object_model.get_part("shoe_head")
    handle = object_model.get_part("swing_handle")
    conduit = object_model.get_part("sample_conduit")
    bend_joint = object_model.get_articulation("shoe_to_swing_handle")

    ctx.check(
        "small class is conduit bender",
        object_model.name == "manual_conduit_bender",
        details=object_model.name,
    )
    ctx.check(
        "primary bend joint is revolute with realistic range",
        bend_joint.articulation_type == ArticulationType.REVOLUTE
        and bend_joint.motion_limits is not None
        and bend_joint.motion_limits.upper >= 1.5,
        details=str(bend_joint),
    )
    for visual_name in (
        "curved_shoe",
        "degree_scale",
        "hook_lip",
        "foot_pedal",
        "serrated_tread_plate",
        "pivot_pin",
    ):
        ctx.check(
            f"shoe has {visual_name}",
            shoe.get_visual(visual_name) is not None,
            details=visual_name,
        )
    for visual_name in ("pivot_bushing", "handle_socket", "handle_tube", "rubber_grip", "pressure_roller"):
        ctx.check(
            f"moving handle has {visual_name}",
            handle.get_visual(visual_name) is not None,
            details=visual_name,
        )
    ctx.check(
        "sample conduit is modeled as a distinct seated tube",
        conduit.get_visual("conduit_tube") is not None,
        details="missing conduit_tube visual",
    )

    # The pin is deliberately captured inside the blue moving bushing, as in a
    # real lever pivot.  This is the only intentional interpenetration.
    ctx.allow_overlap(
        shoe,
        handle,
        elem_a="pivot_pin",
        elem_b="pivot_bushing",
        reason="The steel pivot pin is intentionally represented as captured inside the moving bushing.",
    )
    ctx.expect_within(
        shoe,
        handle,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_bushing",
        margin=0.002,
        name="pivot pin is centered inside the bushing",
    )
    ctx.expect_overlap(
        shoe,
        handle,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_bushing",
        min_overlap=0.025,
        name="pivot pin passes through the bushing width",
    )

    ctx.expect_overlap(
        conduit,
        shoe,
        axes="xz",
        elem_a="conduit_tube",
        elem_b="curved_shoe",
        min_overlap=0.050,
        name="sample conduit follows the curved shoe channel",
    )
    ctx.expect_gap(
        shoe,
        conduit,
        axis="y",
        positive_elem="curved_shoe",
        negative_elem="conduit_tube",
        min_gap=0.004,
        max_gap=0.028,
        name="conduit sits just in front of the shoe groove without colliding",
    )

    rest_aabb = ctx.part_element_world_aabb(handle, elem="handle_tube")
    with ctx.pose({bend_joint: 1.10}):
        posed_aabb = ctx.part_element_world_aabb(handle, elem="handle_tube")
        ctx.expect_overlap(
            handle,
            shoe,
            axes="xz",
            elem_a="pivot_bushing",
            elem_b="pivot_pin",
            min_overlap=0.020,
            name="handle remains mounted on the pivot while bending",
        )
    ctx.check(
        "positive joint raises the handle for bending action",
        rest_aabb is not None
        and posed_aabb is not None
        and posed_aabb[1][2] > rest_aabb[1][2] + 0.22,
        details=f"rest={rest_aabb}, posed={posed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
