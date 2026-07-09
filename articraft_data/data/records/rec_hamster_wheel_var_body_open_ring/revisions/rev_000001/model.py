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
    TorusGeometry,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


WHEEL_RADIUS = 0.112
WHEEL_DEPTH = 0.082
AXLE_HEIGHT = 0.160
AXLE_RADIUS = 0.005


def annular_cylinder_y(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    *,
    segments: int = 96,
) -> MeshGeometry:
    """Closed annular cylinder with its axis along local Y."""
    geom = MeshGeometry()
    y_front = -depth / 2.0
    y_rear = depth / 2.0

    outer_front: list[int] = []
    outer_rear: list[int] = []
    inner_front: list[int] = []
    inner_rear: list[int] = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        x_outer = outer_radius * math.cos(angle)
        z_outer = outer_radius * math.sin(angle)
        x_inner = inner_radius * math.cos(angle)
        z_inner = inner_radius * math.sin(angle)
        outer_front.append(geom.add_vertex(x_outer, y_front, z_outer))
        outer_rear.append(geom.add_vertex(x_outer, y_rear, z_outer))
        inner_front.append(geom.add_vertex(x_inner, y_front, z_inner))
        inner_rear.append(geom.add_vertex(x_inner, y_rear, z_inner))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall
        geom.add_face(outer_front[i], outer_front[j], outer_rear[j])
        geom.add_face(outer_front[i], outer_rear[j], outer_rear[i])
        # Inner wall
        geom.add_face(inner_front[j], inner_front[i], inner_rear[i])
        geom.add_face(inner_front[j], inner_rear[i], inner_rear[j])
        # Front annular lip
        geom.add_face(inner_front[i], inner_front[j], outer_front[j])
        geom.add_face(inner_front[i], outer_front[j], outer_front[i])
        # Rear annular lip
        geom.add_face(outer_rear[i], outer_rear[j], inner_rear[j])
        geom.add_face(outer_rear[i], inner_rear[j], inner_rear[i])

    return geom


def add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str,
) -> None:
    """Add an SDK cylinder whose visual axis is aligned with local/world Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def add_radial_cylinder(
    part,
    *,
    angle: float,
    radius_mid: float,
    length: float,
    rod_radius: float,
    y: float,
    material,
    name: str,
) -> None:
    """Add a cylinder running radially in the local XZ wheel plane."""
    x = radius_mid * math.cos(angle)
    z = radius_mid * math.sin(angle)
    pitch = math.pi / 2.0 - angle
    part.visual(
        Cylinder(radius=rod_radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="transparent_blue_hamster_wheel",
        meta={
            "reference_note": (
                "Hamster removed from the reference image; the asset contains only "
                "the visible exercise wheel and its stand."
            )
        },
    )

    clear_plastic = model.material("clear_plastic", rgba=(0.92, 0.98, 1.0, 0.34))
    blue_plastic = model.material("blue_translucent_plastic", rgba=(0.02, 0.62, 1.0, 0.48))
    blue_edge = model.material("bright_blue_rim", rgba=(0.0, 0.35, 1.0, 0.72))
    white_plastic = model.material("white_axle_cap", rgba=(0.95, 0.96, 0.94, 1.0))

    stand = model.part("stand")
    base_profile = rounded_rect_profile(0.280, 0.150, 0.018, corner_segments=10)
    stand.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(base_profile, 0.010, cap=True, closed=True),
            "rounded_base_plate",
        ),
        origin=Origin(),
        material=clear_plastic,
        name="base_plate",
    )
    stand.visual(
        Box((0.013, 0.012, 0.150)),
        origin=Origin(xyz=(-0.054, 0.058, 0.085)),
        material=clear_plastic,
        name="support_post_0",
    )
    stand.visual(
        Box((0.013, 0.012, 0.150)),
        origin=Origin(xyz=(0.054, 0.058, 0.085)),
        material=clear_plastic,
        name="support_post_1",
    )
    stand.visual(
        Box((0.124, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.058, AXLE_HEIGHT)),
        material=clear_plastic,
        name="bearing_crossbar",
    )
    stand.visual(
        Box((0.128, 0.020, 0.011)),
        origin=Origin(xyz=(0.0, 0.058, 0.0155)),
        material=clear_plastic,
        name="rear_foot_rail",
    )
    add_y_cylinder(
        stand,
        radius=0.017,
        length=0.020,
        xyz=(0.0, 0.064, AXLE_HEIGHT),
        material=blue_plastic,
        name="bearing_block",
    )
    add_y_cylinder(
        stand,
        radius=AXLE_RADIUS,
        length=0.132,
        xyz=(0.0, 0.006, AXLE_HEIGHT),
        material=white_plastic,
        name="axle_shaft",
    )
    add_y_cylinder(
        stand,
        radius=0.014,
        length=0.010,
        xyz=(0.0, -0.064, AXLE_HEIGHT),
        material=white_plastic,
        name="front_axle_cap",
    )

    wheel = model.part("wheel")

    front_rim = TorusGeometry(WHEEL_RADIUS, 0.0045, radial_segments=16, tubular_segments=128)
    front_rim.rotate_x(math.pi / 2.0)
    wheel.visual(
        mesh_from_geometry(front_rim, "rounded_front_rim"),
        origin=Origin(xyz=(0.0, -WHEEL_DEPTH / 2.0, 0.0)),
        material=blue_edge,
        name="front_rim",
    )
    rear_rim = TorusGeometry(WHEEL_RADIUS, 0.0045, radial_segments=16, tubular_segments=128)
    rear_rim.rotate_x(math.pi / 2.0)
    wheel.visual(
        mesh_from_geometry(rear_rim, "rounded_rear_rim"),
        origin=Origin(xyz=(0.0, WHEEL_DEPTH / 2.0, 0.0)),
        material=blue_edge,
        name="rear_rim",
    )

    rear_disk = annular_cylinder_y(WHEEL_RADIUS, AXLE_RADIUS + 0.006, 0.004, segments=128)
    wheel.visual(
        mesh_from_geometry(rear_disk, "rear_disk_with_axle_hole"),
        origin=Origin(xyz=(0.0, -WHEEL_DEPTH / 2.0 + 0.006, 0.0)),
        material=blue_plastic,
        name="rear_disk",
    )

    hub_sleeve = annular_cylinder_y(0.018, AXLE_RADIUS * 0.90, 0.033, segments=72)
    wheel.visual(
        mesh_from_geometry(hub_sleeve, "central_hub_sleeve"),
        origin=Origin(xyz=(0.0, -WHEEL_DEPTH / 2.0 + 0.012, 0.0)),
        material=blue_edge,
        name="wheel_hub",
    )

    # Spokes bridge the hub to the rim so the open-rung wheel reads as one assembly
    spoke_inner = 0.018  # hub outer radius
    spoke_outer = WHEEL_RADIUS  # reach into the rim torus
    spoke_mid = (spoke_inner + spoke_outer) / 2.0
    spoke_len = spoke_outer - spoke_inner
    for i in range(5):
        angle = math.radians(18.0 + i * 72.0)
        add_radial_cylinder(
            wheel,
            angle=angle,
            radius_mid=spoke_mid,
            length=spoke_len,
            rod_radius=0.0028,
            y=-WHEEL_DEPTH / 2.0 + 0.008,
            material=blue_edge,
            name=f"spoke_{i}",
        )

    rung_count = 20
    rung_span = WHEEL_DEPTH - 0.002  # axial span reaching into both rim torus tubes
    rung_radius = 0.0022  # slightly thicker than the old treads for rung visibility
    rung_placement_radius = WHEEL_RADIUS  # at the rim centre-line
    for i in range(rung_count):
        angle = 2.0 * math.pi * i / rung_count
        x = rung_placement_radius * math.cos(angle)
        z = rung_placement_radius * math.sin(angle)
        add_y_cylinder(
            wheel,
            radius=rung_radius,
            length=rung_span,
            xyz=(x, 0.0, z),
            material=blue_edge,
            name=f"rung_{i}",
        )

    model.articulation(
        "stand_to_wheel",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=14.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("stand_to_wheel")

    ctx.allow_overlap(
        stand,
        wheel,
        elem_a="axle_shaft",
        elem_b="wheel_hub",
        reason="The rotating hub sleeve is intentionally captured on the fixed axle shaft.",
    )

    ctx.check(
        "wheel uses a revolute axle joint",
        spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.check(
        "axle joint is centered at wheel height",
        abs(spin.origin.xyz[2] - AXLE_HEIGHT) < 1e-6 and spin.axis == (0.0, 1.0, 0.0),
        details=f"origin={spin.origin}, axis={spin.axis}",
    )

    front_rim_aabb = ctx.part_element_world_aabb(wheel, elem="front_rim")
    if front_rim_aabb is None:
        ctx.fail("front rim is measurable", "front_rim AABB unavailable")
    else:
        (mn, mx) = front_rim_aabb
        rim_dx = mx[0] - mn[0]
        rim_dz = mx[2] - mn[2]
        ctx.check(
            "wheel diameter and roundness are preserved",
            0.220 <= rim_dx <= 0.240 and 0.220 <= rim_dz <= 0.240 and abs(rim_dx - rim_dz) < 0.004,
            details=f"rim_dx={rim_dx:.4f}, rim_dz={rim_dz:.4f}",
        )

    ctx.expect_within(
        stand,
        wheel,
        axes="xz",
        inner_elem="axle_shaft",
        outer_elem="wheel_hub",
        margin=0.0,
        name="axle shaft is coaxial inside hub sleeve",
    )
    ctx.expect_overlap(
        wheel,
        stand,
        axes="y",
        elem_a="wheel_hub",
        elem_b="axle_shaft",
        min_overlap=0.026,
        name="hub remains retained on the shaft",
    )
    ctx.expect_gap(
        stand,
        wheel,
        axis="y",
        positive_elem="bearing_block",
        negative_elem="rear_rim",
        min_gap=0.002,
        name="rear bearing clears the rotating rim",
    )
    ctx.expect_gap(
        wheel,
        stand,
        axis="z",
        positive_elem="front_rim",
        negative_elem="base_plate",
        min_gap=0.025,
        name="wheel stands above base plate",
    )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        mn, mx = box
        return ((mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0, (mn[2] + mx[2]) / 2.0)

    # Verify the open-rung running surface has the expected count and placement
    rung_names = [f"rung_{i}" for i in range(20)]
    rung_aabbs = [ctx.part_element_world_aabb(wheel, elem=n) for n in rung_names]
    ctx.check(
        "wheel has 20 open rungs forming the running surface",
        all(a is not None for a in rung_aabbs),
        details=f"missing_rungs={[n for n, a in zip(rung_names, rung_aabbs) if a is None]}",
    )
    # Confirm rungs sit at the outer wheel radius (open lattice, not an inner drum)
    if rung_aabbs[0] is not None:
        r_mn, r_mx = rung_aabbs[0]
        rung_cx = (r_mn[0] + r_mx[0]) / 2.0
        rung_cz = (r_mn[2] + r_mx[2]) / 2.0
        # Measure from the axle centre (wheel sits at AXLE_HEIGHT on Z, x=0)
        rung_center_xz_dist = math.sqrt(rung_cx ** 2 + (rung_cz - AXLE_HEIGHT) ** 2)
        ctx.check(
            "rung_0 sits at wheel outer radius (open-rung cage)",
            abs(rung_center_xz_dist - WHEEL_RADIUS) < 0.008,
            details=f"rung_r={rung_center_xz_dist:.4f}, expected~{WHEEL_RADIUS}",
        )

    rung_rest = element_center(wheel, "rung_0")
    with ctx.pose({spin: 1.0}):
        rung_rotated = element_center(wheel, "rung_0")
        wheel_pose = ctx.part_world_position(wheel)
    ctx.check(
        "a rung rotates around the axle (open-rung wheel spins)",
        rung_rest is not None
        and rung_rotated is not None
        and abs(rung_rotated[0] - rung_rest[0]) > 0.010
        and abs(rung_rotated[2] - rung_rest[2]) > 0.040,
        details=f"rest={rung_rest}, rotated={rung_rotated}",
    )
    ctx.check(
        "wheel origin stays on axle while spinning",
        wheel_pose is not None
        and abs(wheel_pose[0]) < 1e-6
        and abs(wheel_pose[1]) < 1e-6
        and abs(wheel_pose[2] - AXLE_HEIGHT) < 1e-6,
        details=f"wheel_pose={wheel_pose}",
    )

    return ctx.report()


object_model = build_object_model()
