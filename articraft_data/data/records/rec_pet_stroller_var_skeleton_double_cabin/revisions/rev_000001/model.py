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


def _add_cabin_tier(frame, idx: int, floor_z: float, *,
                    black_fabric, black_plastic, tan_mesh) -> None:
    """Add one fabric cabin compartment: floor, walls, lip, rear panel, mesh windows."""
    p = f"cabin_{idx}"
    wt = 0.030          # floor slab thickness
    wh = 0.190          # side-wall / rear-panel height
    flh = 0.140         # front lip height (shorter opening for pet entry)
    rph = 0.190         # rear panel height (same as walls)

    floor_top = floor_z + wt / 2.0
    wz = floor_top + wh / 2.0          # wall center z
    flz = floor_top + flh / 2.0        # front lip center z
    rpz = floor_top + rph / 2.0        # rear panel center z
    rim_z = wz + wh / 2.0 - 0.015      # front rim sleeve near wall top

    # Floor slab (wide enough to overlap with side walls at x=±0.36)
    frame.visual(
        Box((0.75, 0.88, wt)),
        origin=Origin(xyz=(0.0, 0.0, floor_z)),
        material=black_fabric, name=f"{p}_floor",
    )
    # Side walls (contact floor from above at floor_top)
    frame.visual(
        Box((0.026, 0.91, wh)),
        origin=Origin(xyz=(0.36, 0.0, wz)),
        material=black_fabric, name=f"{p}_side_wall_0",
    )
    frame.visual(
        Box((0.026, 0.91, wh)),
        origin=Origin(xyz=(-0.36, 0.0, wz)),
        material=black_fabric, name=f"{p}_side_wall_1",
    )
    # Front lip (shorter wall with opening above for pet access)
    frame.visual(
        Box((0.75, 0.050, flh)),
        origin=Origin(xyz=(0.0, 0.415, flz)),
        material=black_fabric, name=f"{p}_front_lip",
    )
    # Rear panel
    frame.visual(
        Box((0.75, 0.034, rph)),
        origin=Origin(xyz=(0.0, -0.435, rpz)),
        material=black_fabric, name=f"{p}_rear_panel",
    )
    # Front rim sleeve (rigid trim along the top front edge, wide to reach walls)
    frame.visual(
        Box((0.72, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, 0.430, rim_z)),
        material=black_plastic, name=f"{p}_front_rim_sleeve",
    )
    # Side mesh windows (ventilation panels, centered on side wall face)
    side_panel = _mesh(
        PerforatedPanelGeometry(
            (0.15, 0.28), 0.006, hole_diameter=0.010,
            pitch=(0.020, 0.020), frame=0.010, stagger=True,
        ),
        f"{p}_side_mesh",
    )
    frame.visual(
        side_panel,
        origin=Origin(xyz=(0.366, 0.10, wz), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan_mesh, name=f"{p}_mesh_window_0",
    )
    frame.visual(
        side_panel,
        origin=Origin(xyz=(-0.366, 0.10, wz), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan_mesh, name=f"{p}_mesh_window_1",
    )
    # Front mesh window
    front_panel = _mesh(
        PerforatedPanelGeometry(
            (0.28, 0.12), 0.004, hole_diameter=0.009,
            pitch=(0.018, 0.018), frame=0.010, stagger=True,
        ),
        f"{p}_front_mesh",
    )
    frame.visual(
        front_panel,
        origin=Origin(xyz=(0.0, 0.438, wz - 0.01), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tan_mesh, name=f"{p}_front_mesh_window",
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

    # ── Frame (root) ────────────────────────────────────────────────────
    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.88, 1.35, 1.25)), mass=13.0, origin=Origin(xyz=(0.0, 0.0, 0.62)),
    )

    # Two stacked fabric cabin compartments (lower tier + upper tier)
    for idx, fz in enumerate((0.400, 0.650)):
        _add_cabin_tier(
            frame, idx, fz,
            black_fabric=black_fabric, black_plastic=black_plastic, tan_mesh=tan_mesh,
        )

    # Shelf divider between the two cabin tiers
    frame.visual(
        Box((0.64, 0.86, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=black_fabric, name="cabin_divider",
    )

    # Top rim (raised for double cabin height)
    top_rim_points = [
        (0.35, 0.43, 0.865),
        (0.37, 0.18, 0.880),
        (0.35, -0.42, 0.875),
        (0.00, -0.48, 0.880),
        (-0.35, -0.42, 0.875),
        (-0.37, 0.18, 0.880),
        (-0.35, 0.43, 0.865),
        (0.00, 0.48, 0.865),
        (0.35, 0.43, 0.865),
    ]
    frame.visual(
        _mesh(tube_from_spline_points(
            top_rim_points, radius=0.018, samples_per_segment=16,
            radial_segments=20, closed_spline=True,
        ), "top_rim"),
        material=champagne, name="top_rim",
    )

    # Side rails (raised for double cabin)
    frame.visual(
        _tube([(0.34, 0.43, 0.860), (0.34, 0.08, 0.850), (0.34, -0.32, 0.900), (0.34, -0.47, 1.050)], 0.014, "side_rail_0"),
        material=champagne, name="side_rail_0",
    )
    frame.visual(
        _tube([(-0.34, 0.43, 0.860), (-0.34, 0.08, 0.850), (-0.34, -0.32, 0.900), (-0.34, -0.47, 1.050)], 0.014, "side_rail_1"),
        material=champagne, name="side_rail_1",
    )

    # Lower storage basket (sized to connect storage base to cabin_0 floor)
    frame.visual(Box((0.60, 0.55, 0.026)), origin=Origin(xyz=(0.0, -0.02, 0.260)), material=charcoal_mesh, name="storage_base")
    frame.visual(Box((0.026, 0.55, 0.060)), origin=Origin(xyz=(0.30, -0.02, 0.300)), material=charcoal_mesh, name="storage_side_0")
    frame.visual(Box((0.026, 0.55, 0.060)), origin=Origin(xyz=(-0.30, -0.02, 0.300)), material=charcoal_mesh, name="storage_side_1")
    frame.visual(Box((0.030, 0.035, 0.130)), origin=Origin(xyz=(0.24, -0.02, 0.325)), material=black_fabric, name="storage_hanger_0")
    frame.visual(Box((0.030, 0.035, 0.130)), origin=Origin(xyz=(-0.24, -0.02, 0.325)), material=black_fabric, name="storage_hanger_1")

    # Lower side loops, folding diagonals, rear uprights, fold hinges
    for sx in (-1.0, 1.0):
        x = sx * 0.28
        frame.visual(
            _tube([(x, 0.62, 0.10), (x, 0.38, 0.36), (x, 0.02, 0.46), (x, -0.56, 0.18)], 0.016, f"lower_side_loop_{sx:+.0f}"),
            material=champagne, name=f"lower_side_loop_{0 if sx < 0 else 1}",
        )
        frame.visual(
            _tube([(x, 0.60, 0.20), (x, 0.12, 0.860), (x, -0.38, 0.830)], 0.013, f"folding_diagonal_{sx:+.0f}"),
            material=champagne, name=f"folding_diagonal_{0 if sx < 0 else 1}",
        )
        frame.visual(
            _tube([(x, -0.54, 0.18), (x, -0.38, 0.810), (x, -0.18, 1.050)], 0.015, f"rear_upright_{sx:+.0f}"),
            material=champagne, name=f"rear_upright_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Box((0.055, 0.080, 0.080)),
            origin=Origin(xyz=(x, -0.38, 0.815)),
            material=black_plastic, name=f"fold_hinge_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Cylinder(radius=0.028, length=0.030),
            origin=Origin(xyz=(x, -0.38, 0.815), rpy=(0.0, pi / 2.0, 0.0)),
            material=silver, name=f"hinge_disc_{0 if sx < 0 else 1}",
        )

    # Axle carriers, forks, fenders, visible shock springs
    frame.visual(
        _tube([(-0.34, -0.56, 0.160), (0.34, -0.56, 0.160)], 0.015, "rear_axle_tube"),
        material=black_plastic, name="rear_axle_tube",
    )
    frame.visual(
        _tube([(-0.28, 0.63, 0.110), (0.28, 0.63, 0.110)], 0.013, "front_axle_tube"),
        material=black_plastic, name="front_axle_tube",
    )
    for sx in (-1.0, 1.0):
        x = sx * 0.30
        frame.visual(
            _tube([(x, 0.63, 0.11), (x, 0.62, 0.26), (x, 0.55, 0.35)], 0.018, f"front_fork_{sx:+.0f}"),
            material=black_plastic, name=f"front_fork_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.16),
            origin=Origin(xyz=(x, 0.61, 0.20)),
            material=red_spring, name=f"front_spring_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Box((0.10, 0.11, 0.030)),
            origin=Origin(xyz=(sx * 0.32, 0.62, 0.245)),
            material=black_plastic, name=f"front_fender_clear_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Box((0.060, 0.12, 0.060)),
            origin=Origin(xyz=(sx * 0.325, -0.56, 0.220)),
            material=black_plastic, name=f"rear_axle_bracket_{0 if sx < 0 else 1}",
        )
        # Handle support tubes (raised for double cabin)
        frame.visual(
            _tube([(sx * 0.28, -0.38, 0.830), (sx * 0.44, -0.50, 0.870), (sx * 0.42, -0.70, 0.940), (sx * 0.385, -0.82, 0.910)], 0.014, f"handle_support_{sx:+.0f}"),
            material=champagne, name=f"handle_support_{0 if sx < 0 else 1}",
        )
        frame.visual(
            Box((0.030, 0.052, 0.055)),
            origin=Origin(xyz=(sx * 0.385, -0.82, 0.910)),
            material=black_plastic, name=f"handle_hinge_bracket_{0 if sx < 0 else 1}",
        )

    # Wheel axle stubs
    for idx, (x, y, z, radius, length) in enumerate(
        [
            (-0.385, -0.565, 0.160, 0.006, 0.130),
            (0.385, -0.565, 0.160, 0.006, 0.130),
            (-0.360, 0.630, 0.110, 0.004, 0.130),
            (0.360, 0.630, 0.110, 0.004, 0.130),
        ]
    ):
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=silver, name=f"wheel_axle_stub_{idx}",
        )

    # Handle hinge pins (raised)
    for idx, x in enumerate((-0.33, 0.33)):
        frame.visual(
            Cylinder(radius=0.006, length=0.100),
            origin=Origin(xyz=(x, -0.82, 0.910), rpy=(0.0, pi / 2.0, 0.0)),
            material=silver, name=f"handle_hinge_pin_{idx}",
        )

    # Canopy hinge brackets and pins (raised for double cabin)
    for idx, (x, bx) in enumerate(((-0.35, -0.39), (0.35, 0.39))):
        frame.visual(
            Box((0.035, 0.085, 0.100)),
            origin=Origin(xyz=(bx, -0.485, 0.895)),
            material=black_plastic, name=f"canopy_hinge_bracket_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.006, length=0.100),
            origin=Origin(xyz=(x, -0.520, 0.925), rpy=(0.0, pi / 2.0, 0.0)),
            material=silver, name=f"canopy_hinge_pin_{idx}",
        )

    # ── Canopy (revolute) ───────────────────────────────────────────────
    canopy = model.part("canopy")
    canopy.inertial = Inertial.from_geometry(
        Box((0.74, 0.62, 0.38)), mass=1.2, origin=Origin(xyz=(0.0, 0.22, 0.23)),
    )
    canopy_profile = [(-0.04, 0.00), (-0.06, 0.16), (0.15, 0.35), (0.50, 0.15)]
    canopy.visual(
        _mesh(_canopy_shell_from_side_path(canopy_profile, width=0.72, thickness=0.012), "canopy_fabric"),
        material=black_fabric, name="canopy_fabric",
    )
    for sx in (-1.0, 1.0):
        x = sx * 0.35
        canopy.visual(
            _tube([(x, y, z) for y, z in canopy_profile], 0.014, f"canopy_rib_{sx:+.0f}"),
            material=champagne, name=f"canopy_rib_{0 if sx < 0 else 1}",
        )
        canopy.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=Origin(xyz=(x, -0.02, 0.00), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_plastic, name=f"canopy_hinge_collar_{0 if sx < 0 else 1}",
        )
    canopy.visual(
        _tube([(-0.36, -0.04, 0.00), (0.36, -0.04, 0.00)], 0.012, "canopy_hinge_sleeve"),
        material=champagne, name="canopy_hinge_sleeve",
    )
    canopy.visual(
        _tube([(-0.35, 0.50, 0.15), (0.0, 0.54, 0.16), (0.35, 0.50, 0.15)], 0.012, "canopy_front_bow"),
        material=champagne, name="canopy_front_bow",
    )

    # ── Folding push handle (revolute) ──────────────────────────────────
    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.78, 0.62, 0.56)), mass=1.6, origin=Origin(xyz=(0.0, -0.30, 0.34)),
    )
    handle_points = [
        (-0.33, 0.00, 0.00),
        (-0.34, -0.22, 0.28),
        (-0.31, -0.50, 0.54),
        (-0.16, -0.60, 0.60),
        (0.00, -0.63, 0.61),
        (0.16, -0.60, 0.60),
        (0.31, -0.50, 0.54),
        (0.34, -0.22, 0.28),
        (0.33, 0.00, 0.00),
    ]
    handle.visual(
        _mesh(tube_from_spline_points(
            handle_points, radius=0.016, samples_per_segment=16, radial_segments=20,
        ), "folding_handle_tube"),
        material=champagne, name="handle_tube",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.36),
        origin=Origin(xyz=(0.0, -0.625, 0.61), rpy=(0.0, pi / 2.0, 0.0)),
        material=tan_grip, name="handle_grip",
    )
    for sx in (-1.0, 1.0):
        handle.visual(
            Cylinder(radius=0.029, length=0.026),
            origin=Origin(xyz=(sx * 0.33, 0.00, 0.00), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_plastic, name=f"handle_pivot_{0 if sx < 0 else 1}",
        )
        handle.visual(
            Box((0.055, 0.040, 0.040)),
            origin=Origin(xyz=(sx * 0.31, -0.24, 0.30)),
            material=black_plastic, name=f"handle_clamp_{0 if sx < 0 else 1}",
        )

    # ── Wheels ──────────────────────────────────────────────────────────
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

    # ── Articulations ───────────────────────────────────────────────────
    model.articulation(
        "canopy_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=canopy,
        origin=Origin(xyz=(0.0, -0.50, 0.925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.20, upper=0.90),
    )
    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, -0.82, 0.910)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=0.95),
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
        "frame", "canopy", "handle",
        "front_wheel_0", "front_wheel_1",
        "rear_wheel_0", "rear_wheel_1",
    ]
    ctx.check(
        "pet_stroller_core_parts",
        all(object_model.get_part(name) is not None for name in required_parts),
        f"parts={required_parts!r}",
    )
    required_joints = [
        "canopy_hinge", "handle_hinge",
        "front_wheel_spin_0", "front_wheel_spin_1",
        "rear_wheel_spin_0", "rear_wheel_spin_1",
    ]
    ctx.check(
        "wheel_canopy_handle_joints",
        all(object_model.get_articulation(name) is not None for name in required_joints),
        f"joints={required_joints!r}",
    )

    # Captured-pin overlaps for wheel axles in hubs
    axle_pairs = [
        ("wheel_axle_stub_0", "rear_wheel_0"),
        ("wheel_axle_stub_1", "rear_wheel_1"),
        ("wheel_axle_stub_2", "front_wheel_0"),
        ("wheel_axle_stub_3", "front_wheel_1"),
    ]
    for stub, wheel_name in axle_pairs:
        if frame is not None and object_model.get_part(wheel_name) is not None:
            ctx.allow_overlap(
                "frame", wheel_name,
                elem_a=stub, elem_b="hub_cap",
                reason="The small axle stub is intentionally captured inside the spinning wheel hub.",
            )
            ctx.expect_overlap(
                "frame", wheel_name,
                axes="xyz", elem_a=stub, elem_b="hub_cap",
                min_overlap=0.004, name=f"{stub}_captured_in_hub",
            )

    # Handle hinge pin overlaps
    if frame is not None and handle is not None:
        for idx in (0, 1):
            ctx.allow_overlap(
                "frame", "handle",
                elem_a=f"handle_hinge_pin_{idx}", elem_b=f"handle_pivot_{idx}",
                reason="The hinge pin is intentionally captured inside the folding handle pivot collar.",
            )
            ctx.allow_overlap(
                "frame", "handle",
                elem_a=f"handle_hinge_pin_{idx}", elem_b="handle_tube",
                reason="The hinge pin passes through the end of the handle tube at the pivot.",
            )
            ctx.expect_overlap(
                "frame", "handle",
                axes="xyz",
                elem_a=f"handle_hinge_pin_{idx}", elem_b=f"handle_pivot_{idx}",
                min_overlap=0.004, name=f"handle_hinge_pin_{idx}_captured",
            )

    # Canopy hinge pin overlaps
    if frame is not None and canopy is not None:
        for idx in (0, 1):
            ctx.allow_overlap(
                "frame", "canopy",
                elem_a=f"canopy_hinge_pin_{idx}", elem_b=f"canopy_hinge_collar_{idx}",
                reason="The canopy hinge pin is intentionally captured inside the folding canopy collar.",
            )
            ctx.expect_overlap(
                "frame", "canopy",
                axes="xyz",
                elem_a=f"canopy_hinge_pin_{idx}", elem_b=f"canopy_hinge_collar_{idx}",
                min_overlap=0.004, name=f"canopy_hinge_pin_{idx}_captured",
            )

    # Canopy position relative to upper cabin
    if frame is not None and canopy is not None:
        ctx.expect_overlap(canopy, frame, axes="xy", min_overlap=0.30, name="canopy_overlaps_basket_footprint")
        # Canopy must sit well above the upper cabin floor (z > 0.65)
        canopy_aabb = ctx.part_world_aabb(canopy)
        if canopy_aabb is not None:
            ctx.check(
                "canopy_sits_above_upper_cabin",
                float(canopy_aabb[0][2]) > 0.80,
                details=f"canopy_aabb={canopy_aabb!r}",
            )

    # Wheel ground contact and proportions
    for wheel_name in ("front_wheel_0", "front_wheel_1", "rear_wheel_0", "rear_wheel_1"):
        wheel = object_model.get_part(wheel_name)
        if wheel is not None:
            aabb = ctx.part_world_aabb(wheel)
            if aabb is not None:
                mins, maxs = aabb
                ctx.check(
                    f"{wheel_name}_contacts_ground",
                    abs(float(mins[2])) <= 0.008,
                    details=f"aabb={aabb!r}",
                )
                ctx.check(
                    f"{wheel_name}_round_proportions",
                    float(maxs[2] - mins[2]) > 0.18 if "front" in wheel_name else float(maxs[2] - mins[2]) > 0.28,
                    details=f"aabb={aabb!r}",
                )

    # Canopy hinge articulation test
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

    # Handle hinge articulation test
    handle_hinge = object_model.get_articulation("handle_hinge")
    if handle is not None and handle_hinge is not None:
        rest_aabb = ctx.part_world_aabb(handle)
        with ctx.pose({handle_hinge: 0.85}):
            folded_aabb = ctx.part_world_aabb(handle)
        if rest_aabb is not None and folded_aabb is not None:
            ctx.check(
                "handle_folds_downward",
                float(folded_aabb[1][2]) < float(rest_aabb[1][2]) - 0.12,
                details=f"rest={rest_aabb!r}, folded={folded_aabb!r}",
            )

    # ── Double cabin structure tests ────────────────────────────────────
    if frame is not None:
        c0_floor_aabb = ctx.part_element_world_aabb(frame, elem="cabin_0_floor")
        c1_floor_aabb = ctx.part_element_world_aabb(frame, elem="cabin_1_floor")
        divider_aabb = ctx.part_element_world_aabb(frame, elem="cabin_divider")

        if c0_floor_aabb is not None and c1_floor_aabb is not None:
            # Upper cabin floor is vertically above the lower cabin top
            z_gap = float(c1_floor_aabb[0][2]) - float(c0_floor_aabb[1][2])
            ctx.check(
                "upper_cabin_above_lower_cabin",
                z_gap > 0.005,
                details=f"c0_floor_aabb={c0_floor_aabb!r}, c1_floor_aabb={c1_floor_aabb!r}, z_gap={z_gap:.4f}",
            )
            # Both cabin tiers share the same XY footprint
            x_overlap = min(float(c0_floor_aabb[1][0]), float(c1_floor_aabb[1][0])) - max(float(c0_floor_aabb[0][0]), float(c1_floor_aabb[0][0]))
            y_overlap = min(float(c0_floor_aabb[1][1]), float(c1_floor_aabb[1][1])) - max(float(c0_floor_aabb[0][1]), float(c1_floor_aabb[0][1]))
            ctx.check(
                "both_cabins_share_footprint",
                x_overlap > 0.60 and y_overlap > 0.80,
                details=f"x_overlap={x_overlap:.4f}, y_overlap={y_overlap:.4f}",
            )

        if divider_aabb is not None and c0_floor_aabb is not None and c1_floor_aabb is not None:
            # Divider sits between the two tiers
            ctx.check(
                "divider_between_tiers",
                float(divider_aabb[0][2]) >= float(c0_floor_aabb[1][2]) - 0.005
                and float(divider_aabb[1][2]) <= float(c1_floor_aabb[0][2]) + 0.005,
                details=f"divider={divider_aabb!r}, c0={c0_floor_aabb!r}, c1={c1_floor_aabb!r}",
            )

    return ctx.report()


object_model = build_object_model()
