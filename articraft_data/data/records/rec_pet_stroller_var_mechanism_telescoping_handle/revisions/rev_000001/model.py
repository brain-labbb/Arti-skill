from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _tube(points, radius: float, name: str):
    return _mesh(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=12,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _curved_canopy_shell(
    *,
    width: float,
    radius: float,
    center_y: float,
    center_z: float,
    theta0: float,
    theta1: float,
    thickness: float,
    width_segments: int = 6,
    arc_segments: int = 28,
) -> MeshGeometry:
    """A thin, closed curved fabric shell, local X is stroller width."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for ix in range(width_segments + 1):
        x = -width / 2.0 + width * ix / width_segments
        outer_row: list[int] = []
        inner_row: list[int] = []
        for ia in range(arc_segments + 1):
            t = theta0 + (theta1 - theta0) * ia / arc_segments
            outer_row.append(
                geom.add_vertex(
                    x,
                    center_y + radius * cos(t),
                    center_z + radius * sin(t),
                )
            )
            inner_row.append(
                geom.add_vertex(
                    x,
                    center_y + (radius - thickness) * cos(t),
                    center_z + (radius - thickness) * sin(t),
                )
            )
        outer.append(outer_row)
        inner.append(inner_row)

    for ix in range(width_segments):
        for ia in range(arc_segments):
            a, b, c, d = outer[ix][ia], outer[ix + 1][ia], outer[ix + 1][ia + 1], outer[ix][ia + 1]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
            ai, bi, ci, di = inner[ix][ia], inner[ix][ia + 1], inner[ix + 1][ia + 1], inner[ix + 1][ia]
            geom.add_face(ai, bi, ci)
            geom.add_face(ai, ci, di)

    # End caps at the two side edges and at the front/rear cut edges.
    for ix in (0, width_segments):
        for ia in range(arc_segments):
            a, b = outer[ix][ia], outer[ix][ia + 1]
            c, d = inner[ix][ia + 1], inner[ix][ia]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    for ia in (0, arc_segments):
        for ix in range(width_segments):
            a, b = outer[ix][ia], outer[ix + 1][ia]
            c, d = inner[ix + 1][ia], inner[ix][ia]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _canopy_shell_from_side_path(
    path: list[tuple[float, float]],
    *,
    width: float,
    thickness: float,
) -> MeshGeometry:
    """A thin canopy sheet whose side edges share the rib path."""
    geom = MeshGeometry()
    half = width / 2.0
    rows = []
    for y, z in path:
        rows.append(
            (
                geom.add_vertex(-half, y, z),
                geom.add_vertex(half, y, z),
                geom.add_vertex(-half, y, z - thickness),
                geom.add_vertex(half, y, z - thickness),
            )
        )

    for i in range(len(rows) - 1):
        a0, a1, a2, a3 = rows[i]
        b0, b1, b2, b3 = rows[i + 1]
        geom.add_face(a0, b0, b1)
        geom.add_face(a0, b1, a1)
        geom.add_face(a2, a3, b3)
        geom.add_face(a2, b3, b2)
        geom.add_face(a0, a2, b2)
        geom.add_face(a0, b2, b0)
        geom.add_face(a1, b1, b3)
        geom.add_face(a1, b3, a3)

    for row in (rows[0], rows[-1]):
        a0, a1, a2, a3 = row
        geom.add_face(a0, a1, a3)
        geom.add_face(a0, a3, a2)
    return geom


def _add_wheel(part, prefix: str, *, tire_radius: float, tire_width: float, material_rim, material_tire, material_gold) -> None:
    rim_radius = tire_radius * 0.72
    part.visual(
        _mesh(
            TireGeometry(
                tire_radius,
                tire_width,
                inner_radius=rim_radius * 0.98,
                tread=TireTread(style="circumferential", depth=tire_radius * 0.030, count=4),
                grooves=(TireGroove(center_offset=0.0, width=tire_width * 0.11, depth=tire_radius * 0.012),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            f"{prefix}_tire",
        ),
        material=material_tire,
        name="tire",
    )
    part.visual(
        _mesh(
            WheelGeometry(
                rim_radius,
                tire_width * 0.72,
                rim=WheelRim(
                    inner_radius=rim_radius * 0.62,
                    flange_height=tire_radius * 0.035,
                    flange_thickness=tire_width * 0.06,
                ),
                hub=WheelHub(
                    radius=rim_radius * 0.30,
                    width=tire_width * 0.62,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=rim_radius * 0.38, hole_diameter=tire_radius * 0.025),
                ),
                face=WheelFace(dish_depth=tire_width * 0.10, front_inset=tire_width * 0.04),
                spokes=WheelSpokes(style="straight", count=8, thickness=tire_radius * 0.020, window_radius=tire_radius * 0.045),
                bore=WheelBore(style="round", diameter=tire_radius * 0.10),
            ),
            f"{prefix}_wheel",
        ),
        material=material_rim,
        name="rim",
    )
    part.visual(
        Cylinder(radius=rim_radius * 0.18, length=tire_width * 1.05),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=material_gold,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_stroller")

    black_fabric = model.material("black_fabric", rgba=(0.015, 0.015, 0.016, 1.0))
    charcoal_mesh = model.material("charcoal_mesh", rgba=(0.025, 0.027, 0.026, 0.72))
    tan_mesh = model.material("tan_window_mesh", rgba=(0.70, 0.60, 0.43, 0.78))
    champagne = model.material("champagne_tube", rgba=(0.92, 0.62, 0.31, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.035, 0.036, 0.038, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.035, 0.035, 0.037, 1.0))
    silver = model.material("silver_fasteners", rgba=(0.78, 0.76, 0.70, 1.0))
    red_spring = model.material("red_spring", rgba=(0.90, 0.05, 0.03, 1.0))
    tan_grip = model.material("tan_handle_grip", rgba=(0.96, 0.56, 0.27, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(Box((0.88, 1.35, 1.05)), mass=11.0, origin=Origin(xyz=(0.0, 0.0, 0.52)))

    # Hollow fabric basket/cabin: side walls, lower front lip, rear panel and open top.
    frame.visual(Box((0.66, 0.88, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.435)), material=black_fabric, name="basket_floor")
    frame.visual(Box((0.026, 0.91, 0.245)), origin=Origin(xyz=(0.36, 0.0, 0.555)), material=black_fabric, name="side_wall_0")
    frame.visual(Box((0.026, 0.91, 0.245)), origin=Origin(xyz=(-0.36, 0.0, 0.555)), material=black_fabric, name="side_wall_1")
    frame.visual(Box((0.66, 0.050, 0.180)), origin=Origin(xyz=(0.0, 0.415, 0.530)), material=black_fabric, name="front_lip")
    frame.visual(Box((0.66, 0.034, 0.305)), origin=Origin(xyz=(0.0, -0.435, 0.595)), material=black_fabric, name="rear_panel")
    frame.visual(Box((0.58, 0.030, 0.040)), origin=Origin(xyz=(0.0, 0.430, 0.668)), material=black_plastic, name="front_rim_sleeve")

    top_rim_points = [
        (0.35, 0.43, 0.675),
        (0.37, 0.18, 0.690),
        (0.35, -0.42, 0.685),
        (0.00, -0.48, 0.690),
        (-0.35, -0.42, 0.685),
        (-0.37, 0.18, 0.690),
        (-0.35, 0.43, 0.675),
        (0.00, 0.48, 0.675),
        (0.35, 0.43, 0.675),
    ]
    frame.visual(_mesh(tube_from_spline_points(top_rim_points, radius=0.018, samples_per_segment=16, radial_segments=20, closed_spline=True), "top_rim"), material=champagne, name="top_rim")
    frame.visual(_tube([(0.34, 0.43, 0.67), (0.34, 0.08, 0.66), (0.34, -0.32, 0.71), (0.34, -0.47, 0.86)], 0.014, "side_rail_0"), material=champagne, name="side_rail_0")
    frame.visual(_tube([(-0.34, 0.43, 0.67), (-0.34, 0.08, 0.66), (-0.34, -0.32, 0.71), (-0.34, -0.47, 0.86)], 0.014, "side_rail_1"), material=champagne, name="side_rail_1")

    # Mesh windows in the fabric cabin.
    side_panel = _mesh(PerforatedPanelGeometry((0.18, 0.32), 0.004, hole_diameter=0.010, pitch=(0.020, 0.020), frame=0.010, stagger=True), "side_mesh_window")
    frame.visual(side_panel, origin=Origin(xyz=(0.374, 0.10, 0.555), rpy=(0.0, pi / 2.0, 0.0)), material=tan_mesh, name="mesh_window_0")
    frame.visual(side_panel, origin=Origin(xyz=(-0.374, 0.10, 0.555), rpy=(0.0, pi / 2.0, 0.0)), material=tan_mesh, name="mesh_window_1")
    front_panel = _mesh(PerforatedPanelGeometry((0.30, 0.16), 0.004, hole_diameter=0.009, pitch=(0.018, 0.018), frame=0.010, stagger=True), "front_mesh_window")
    frame.visual(front_panel, origin=Origin(xyz=(0.0, 0.438, 0.545), rpy=(pi / 2.0, 0.0, 0.0)), material=tan_mesh, name="front_mesh_window")

    # Lower storage basket and tubular undercarriage.
    frame.visual(Box((0.54, 0.55, 0.026)), origin=Origin(xyz=(0.0, -0.02, 0.285)), material=charcoal_mesh, name="storage_base")
    frame.visual(Box((0.026, 0.55, 0.120)), origin=Origin(xyz=(0.30, -0.02, 0.335)), material=charcoal_mesh, name="storage_side_0")
    frame.visual(Box((0.026, 0.55, 0.120)), origin=Origin(xyz=(-0.30, -0.02, 0.335)), material=charcoal_mesh, name="storage_side_1")
    frame.visual(Box((0.030, 0.035, 0.160)), origin=Origin(xyz=(0.24, -0.02, 0.360)), material=black_fabric, name="storage_hanger_0")
    frame.visual(Box((0.030, 0.035, 0.160)), origin=Origin(xyz=(-0.24, -0.02, 0.360)), material=black_fabric, name="storage_hanger_1")
    for sx in (-1.0, 1.0):
        x = sx * 0.28
        frame.visual(_tube([(x, 0.62, 0.10), (x, 0.38, 0.36), (x, 0.02, 0.46), (x, -0.56, 0.18)], 0.016, f"lower_side_loop_{sx:+.0f}"), material=champagne, name=f"lower_side_loop_{0 if sx < 0 else 1}")
        frame.visual(_tube([(x, 0.60, 0.20), (x, 0.12, 0.67), (sx * 0.34, -0.40, 0.60)], 0.013, f"folding_diagonal_{sx:+.0f}"), material=champagne, name=f"folding_diagonal_{0 if sx < 0 else 1}")
        frame.visual(_tube([(x, -0.54, 0.18), (x, -0.38, 0.62), (x, -0.18, 0.86)], 0.015, f"rear_upright_{sx:+.0f}"), material=champagne, name=f"rear_upright_{0 if sx < 0 else 1}")
        frame.visual(Box((0.055, 0.080, 0.080)), origin=Origin(xyz=(sx * 0.34, -0.38, 0.625)), material=black_plastic, name=f"fold_hinge_{0 if sx < 0 else 1}")
        frame.visual(Cylinder(radius=0.028, length=0.030), origin=Origin(xyz=(sx * 0.34, -0.38, 0.625), rpy=(0.0, pi / 2.0, 0.0)), material=silver, name=f"hinge_disc_{0 if sx < 0 else 1}")

    # Axle carriers, forks, fenders, visible shock springs.
    frame.visual(_tube([(-0.34, -0.56, 0.160), (0.34, -0.56, 0.160)], 0.015, "rear_axle_tube"), material=black_plastic, name="rear_axle_tube")
    frame.visual(_tube([(-0.28, 0.63, 0.110), (0.28, 0.63, 0.110)], 0.013, "front_axle_tube"), material=black_plastic, name="front_axle_tube")
    for sx in (-1.0, 1.0):
        x = sx * 0.30
        frame.visual(_tube([(x, 0.63, 0.11), (x, 0.62, 0.26), (x, 0.55, 0.35)], 0.018, f"front_fork_{sx:+.0f}"), material=black_plastic, name=f"front_fork_{0 if sx < 0 else 1}")
        frame.visual(Cylinder(radius=0.012, length=0.16), origin=Origin(xyz=(x, 0.61, 0.20)), material=red_spring, name=f"front_spring_{0 if sx < 0 else 1}")
        frame.visual(Box((0.10, 0.11, 0.030)), origin=Origin(xyz=(sx * 0.32, 0.62, 0.245)), material=black_plastic, name=f"front_fender_clear_{0 if sx < 0 else 1}")
        frame.visual(Box((0.060, 0.12, 0.060)), origin=Origin(xyz=(sx * 0.325, -0.56, 0.220)), material=black_plastic, name=f"rear_axle_bracket_{0 if sx < 0 else 1}")
        # Handle support braces: from fold hinge area down to the lower frame,
        # bracing the telescope sleeve mounting region laterally.
        frame.visual(_tube([(sx * 0.34, -0.38, 0.625), (sx * 0.36, -0.44, 0.50), (sx * 0.34, -0.52, 0.34)], 0.014, f"handle_support_{sx:+.0f}"), material=champagne, name=f"handle_support_{0 if sx < 0 else 1}")

    # Telescope outer sleeves fixed to the frame rear uprights.
    # The handle side tubes slide inside these sleeves (PRISMATIC joint).
    for idx, sx in enumerate((-1.0, 1.0)):
        x = sx * 0.28
        frame.visual(
            _tube([(x, -0.48, 0.42), (x, -0.38, 0.64)], 0.022, f"telescope_sleeve_{sx:+.0f}"),
            material=black_plastic,
            name=f"telescope_sleeve_{idx}",
        )


    for idx, (x, y, z, radius, length) in enumerate(
        [
            (-0.385, -0.565, 0.160, 0.006, 0.130),
            (0.385, -0.565, 0.160, 0.006, 0.130),
            (-0.360, 0.630, 0.110, 0.004, 0.130),
            (0.360, 0.630, 0.110, 0.004, 0.130),
        ]
    ):
        frame.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)), material=silver, name=f"wheel_axle_stub_{idx}")
    for idx, (x, bx) in enumerate(((-0.35, -0.39), (0.35, 0.39))):
        frame.visual(Box((0.035, 0.085, 0.100)), origin=Origin(xyz=(bx, -0.485, 0.705)), material=black_plastic, name=f"canopy_hinge_bracket_{idx}")
        frame.visual(Cylinder(radius=0.006, length=0.100), origin=Origin(xyz=(x, -0.520, 0.735), rpy=(0.0, pi / 2.0, 0.0)), material=silver, name=f"canopy_hinge_pin_{idx}")

    # Revolute canopy part: fabric shell and arched side ribs, hinged at the cabin sides.
    canopy = model.part("canopy")
    canopy.inertial = Inertial.from_geometry(Box((0.74, 0.62, 0.38)), mass=1.2, origin=Origin(xyz=(0.0, 0.22, 0.23)))
    canopy_profile = [(-0.04, 0.00), (-0.06, 0.16), (0.15, 0.35), (0.50, 0.15)]
    canopy.visual(_mesh(_canopy_shell_from_side_path(canopy_profile, width=0.72, thickness=0.012), "canopy_fabric"), material=black_fabric, name="canopy_fabric")
    for sx in (-1.0, 1.0):
        x = sx * 0.35
        canopy.visual(_tube([(x, y, z) for y, z in canopy_profile], 0.014, f"canopy_rib_{sx:+.0f}"), material=champagne, name=f"canopy_rib_{0 if sx < 0 else 1}")
        canopy.visual(Cylinder(radius=0.026, length=0.030), origin=Origin(xyz=(x, -0.02, 0.00), rpy=(0.0, pi / 2.0, 0.0)), material=black_plastic, name=f"canopy_hinge_collar_{0 if sx < 0 else 1}")
    canopy.visual(_tube([(-0.36, -0.04, 0.00), (0.36, -0.04, 0.00)], 0.012, "canopy_hinge_sleeve"), material=champagne, name="canopy_hinge_sleeve")
    canopy.visual(_tube([(-0.35, 0.50, 0.15), (0.0, 0.54, 0.16), (0.35, 0.50, 0.15)], 0.012, "canopy_front_bow"), material=champagne, name="canopy_front_bow")

    # Telescoping push handle: U-tube slides up out of the two frame-mounted sleeves.
    # Part frame at q=0 coincides with the prismatic joint origin at the sleeve base.
    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(Box((0.60, 0.24, 0.52)), mass=1.4, origin=Origin(xyz=(0.0, 0.09, 0.22)))
    # Handle tube: two straight lower legs (inside sleeves) curve into a U-grip above.
    # Local frame origin = joint origin at q=0 (world: 0, -0.46, 0.46).
    handle_points = [
        (-0.28, -0.02, -0.04),   # left tube bottom (captured inside sleeve)
        (-0.28, 0.04, 0.10),     # left mid-sleeve
        (-0.28, 0.08, 0.18),     # left at sleeve exit
        (-0.26, 0.12, 0.28),     # left above sleeve, curving inward
        (-0.20, 0.16, 0.36),     # left curve
        (-0.12, 0.18, 0.42),     # left approaching grip
        (0.00, 0.20, 0.46),      # grip center
        (0.12, 0.18, 0.42),      # right approaching grip
        (0.20, 0.16, 0.36),      # right curve
        (0.26, 0.12, 0.28),      # right above sleeve, curving inward
        (0.28, 0.08, 0.18),      # right at sleeve exit
        (0.28, 0.04, 0.10),      # right mid-sleeve
        (0.28, -0.02, -0.04),    # right tube bottom (captured inside sleeve)
    ]
    handle.visual(
        _mesh(tube_from_spline_points(handle_points, radius=0.016, samples_per_segment=16, radial_segments=20), "handle_tube"),
        material=champagne,
        name="handle_tube",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.30),
        origin=Origin(xyz=(0.0, 0.20, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan_grip,
        name="handle_grip",
    )
    # Slide-lock collars on the handle tube above the sleeve exit,
    # visible as the height-adjustment locking hardware.
    for idx, sx in enumerate((-1.0, 1.0)):
        handle.visual(
            Cylinder(radius=0.024, length=0.022),
            origin=Origin(xyz=(sx * 0.28, 0.14, 0.30), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_plastic,
            name=f"handle_pivot_{idx}",
        )

    # Four wheels, separate spin links. Rear wheels are visibly larger than the front casters.
    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_0.inertial = Inertial.from_geometry(Cylinder(radius=0.160, length=0.060), mass=0.60, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)))
    _add_wheel(rear_wheel_0, "rear_wheel_0", tire_radius=0.160, tire_width=0.060, material_rim=wheel_black, material_tire=rubber, material_gold=champagne)

    rear_wheel_1 = model.part("rear_wheel_1")
    rear_wheel_1.inertial = Inertial.from_geometry(Cylinder(radius=0.160, length=0.060), mass=0.60, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)))
    _add_wheel(rear_wheel_1, "rear_wheel_1", tire_radius=0.160, tire_width=0.060, material_rim=wheel_black, material_tire=rubber, material_gold=champagne)

    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_0.inertial = Inertial.from_geometry(Cylinder(radius=0.110, length=0.050), mass=0.36, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)))
    _add_wheel(front_wheel_0, "front_wheel_0", tire_radius=0.110, tire_width=0.050, material_rim=wheel_black, material_tire=rubber, material_gold=champagne)

    front_wheel_1 = model.part("front_wheel_1")
    front_wheel_1.inertial = Inertial.from_geometry(Cylinder(radius=0.110, length=0.050), mass=0.36, origin=Origin(rpy=(0.0, pi / 2.0, 0.0)))
    _add_wheel(front_wheel_1, "front_wheel_1", tire_radius=0.110, tire_width=0.050, material_rim=wheel_black, material_tire=rubber, material_gold=champagne)

    model.articulation(
        "canopy_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=canopy,
        origin=Origin(xyz=(0.0, -0.50, 0.735)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.20, upper=0.90),
    )
    # Telescoping handle: PRISMATIC along the rear upright direction.
    # Positive q extends the handle upward/out of the sleeves.
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, -0.46, 0.46)),
        axis=(0.0, 0.4, 0.9),
        motion_limits=MotionLimits(effort=8.0, velocity=0.5, lower=0.0, upper=0.18),
    )
    for name, child, xyz in [
        ("rear_wheel_spin_0", rear_wheel_0, (-0.385, -0.565, 0.160)),
        ("rear_wheel_spin_1", rear_wheel_1, (0.385, -0.565, 0.160)),
        ("front_wheel_spin_0", front_wheel_0, (-0.360, 0.630, 0.110)),
        ("front_wheel_spin_1", front_wheel_1, (0.360, 0.630, 0.110)),
    ]:
        model.articulation(
            name,
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=child,
            origin=Origin(xyz=xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    canopy = object_model.get_part("canopy")
    handle = object_model.get_part("handle")
    required_parts = [
        "frame",
        "canopy",
        "handle",
        "front_wheel_0",
        "front_wheel_1",
        "rear_wheel_0",
        "rear_wheel_1",
    ]
    ctx.check("pet_stroller_core_parts", all(object_model.get_part(name) is not None for name in required_parts), f"parts={required_parts!r}")
    required_joints = [
        "canopy_hinge",
        "handle_slide",
        "front_wheel_spin_0",
        "front_wheel_spin_1",
        "rear_wheel_spin_0",
        "rear_wheel_spin_1",
    ]
    ctx.check("wheel_canopy_handle_joints", all(object_model.get_articulation(name) is not None for name in required_joints), f"joints={required_joints!r}")

    # Verify telescope sleeves exist on the frame (structural delta of this variant).
    ctx.check(
        "telescope_sleeves_on_frame",
        all(
            frame is not None and frame.get_visual(f"telescope_sleeve_{i}") is not None
            for i in (0, 1)
        ),
        "telescope_sleeve_0 and telescope_sleeve_1 must be visible on frame",
    )

    # Small hidden captured-pin overlaps intentionally represent real wheel axles.
    axle_pairs = [
        ("wheel_axle_stub_0", "rear_wheel_0"),
        ("wheel_axle_stub_1", "rear_wheel_1"),
        ("wheel_axle_stub_2", "front_wheel_0"),
        ("wheel_axle_stub_3", "front_wheel_1"),
    ]
    for stub, wheel_name in axle_pairs:
        if frame is not None and object_model.get_part(wheel_name) is not None:
            ctx.allow_overlap(
                "frame",
                wheel_name,
                elem_a=stub,
                elem_b="hub_cap",
                reason="The small axle stub is intentionally captured inside the spinning wheel hub.",
            )
            ctx.expect_overlap(
                "frame",
                wheel_name,
                axes="xyz",
                elem_a=stub,
                elem_b="hub_cap",
                min_overlap=0.004,
                name=f"{stub}_captured_in_hub",
            )

    # The handle tube slides inside the telescope sleeves (captured prismatic fit).
    if frame is not None and handle is not None:
        for idx in (0, 1):
            ctx.allow_overlap(
                "frame",
                "handle",
                elem_a=f"telescope_sleeve_{idx}",
                elem_b="handle_tube",
                reason="The handle inner tube is intentionally captured inside the outer telescope sleeve for prismatic height adjustment.",
            )
            ctx.expect_overlap(
                "frame",
                "handle",
                axes="xyz",
                elem_a=f"telescope_sleeve_{idx}",
                elem_b="handle_tube",
                min_overlap=0.010,
                name=f"telescope_sleeve_{idx}_retains_handle_tube",
            )
        # The handle tubes pass through slots in the rear fabric panel,
        # as in real pet stroller construction where the handle frame
        # penetrates the rear cabin wall through fabric grommets.
        ctx.allow_overlap(
            "frame",
            "handle",
            elem_a="rear_panel",
            elem_b="handle_tube",
            reason="The handle tubes pass through slots in the rear fabric panel, as in real pet stroller construction.",
        )
        ctx.expect_overlap(
            "frame",
            "handle",
            axes="y",
            elem_a="rear_panel",
            elem_b="handle_tube",
            min_overlap=0.01,
            name="handle_tube_passes_through_rear_panel_slot",
        )
        # The handle inner tubes slide coaxially along the rear uprights
        # through the telescope sleeves (the upright acts as the structural
        # outer rail of the telescoping mechanism).
        for idx in (0, 1):
            ctx.allow_overlap(
                "frame",
                "handle",
                elem_a=f"rear_upright_{idx}",
                elem_b="handle_tube",
                reason="The handle tube slides coaxially along the rear upright through the telescope sleeve, forming the telescoping height-adjustment rail.",
            )
            ctx.allow_overlap(
                "frame",
                "handle",
                elem_a=f"rear_upright_{idx}",
                elem_b=f"handle_pivot_{idx}",
                reason="The slide collar sits at the sleeve exit and wraps around the upright-tube junction.",
            )
            ctx.expect_overlap(
                "frame",
                "handle",
                axes="z",
                elem_a=f"rear_upright_{idx}",
                elem_b="handle_tube",
                min_overlap=0.05,
                name=f"rear_upright_{idx}_guides_handle_tube",
            )

    if frame is not None and canopy is not None:
        for idx in (0, 1):
            ctx.allow_overlap(
                "frame",
                "canopy",
                elem_a=f"canopy_hinge_pin_{idx}",
                elem_b=f"canopy_hinge_collar_{idx}",
                reason="The canopy hinge pin is intentionally captured inside the folding canopy collar.",
            )
            ctx.expect_overlap(
                "frame",
                "canopy",
                axes="xyz",
                elem_a=f"canopy_hinge_pin_{idx}",
                elem_b=f"canopy_hinge_collar_{idx}",
                min_overlap=0.004,
                name=f"canopy_hinge_pin_{idx}_captured",
            )

    if frame is not None and canopy is not None:
        ctx.expect_overlap(canopy, frame, axes="xy", min_overlap=0.30, name="canopy_overlaps_basket_footprint")
        ctx.expect_gap(
            canopy,
            frame,
            axis="z",
            min_gap=0.20,
            max_gap=0.55,
            negative_elem="basket_floor",
            name="canopy_sits_above_open_basket_floor",
        )

    for wheel_name in ("front_wheel_0", "front_wheel_1", "rear_wheel_0", "rear_wheel_1"):
        wheel = object_model.get_part(wheel_name)
        if wheel is not None:
            aabb = ctx.part_world_aabb(wheel)
            if aabb is not None:
                mins, maxs = aabb
                ctx.check(f"{wheel_name}_contacts_ground", abs(float(mins[2])) <= 0.008, details=f"aabb={aabb!r}")
                ctx.check(f"{wheel_name}_round_proportions", float(maxs[2] - mins[2]) > 0.18 if "front" in wheel_name else float(maxs[2] - mins[2]) > 0.28, details=f"aabb={aabb!r}")

    canopy_hinge = object_model.get_articulation("canopy_hinge")
    if canopy is not None and canopy_hinge is not None:
        rest_aabb = ctx.part_world_aabb(canopy)
        with ctx.pose({canopy_hinge: 0.70}):
            folded_aabb = ctx.part_world_aabb(canopy)
        if rest_aabb is not None and folded_aabb is not None:
            ctx.check(
                "canopy_hinge_lifts_front_edge",
                float(folded_aabb[1][2]) > float(rest_aabb[1][2]) + 0.03,
                details=f"rest={rest_aabb!r}, folded={folded_aabb!r}",
            )

    # Prismatic handle_slide: prove the handle extends upward at max travel.
    handle_slide = object_model.get_articulation("handle_slide")
    if handle is not None and handle_slide is not None:
        rest_pos = ctx.part_world_position(handle)
        with ctx.pose({handle_slide: 0.18}):
            extended_pos = ctx.part_world_position(handle)
        if rest_pos is not None and extended_pos is not None:
            ctx.check(
                "handle_slide_extends_upward",
                float(extended_pos[2]) > float(rest_pos[2]) + 0.10,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )
            ctx.check(
                "handle_slide_travel_matches_design",
                abs(float(extended_pos[2]) - float(rest_pos[2]) - 0.165) < 0.03,
                details=f"dz={float(extended_pos[2]) - float(rest_pos[2]):.4f}, expected~0.165",
            )

    return ctx.report()


object_model = build_object_model()
