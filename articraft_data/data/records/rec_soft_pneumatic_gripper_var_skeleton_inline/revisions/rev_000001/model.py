from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 28, center: tuple[float, float] = (0.0, 0.0)):
    cx, cy = center
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _rot_xy(r: float, yaw: float) -> tuple[float, float]:
    """Rotate (r, 0) by yaw in the XY plane. Preserved helper from parent baseline."""
    return (r * math.cos(yaw), r * math.sin(yaw))


def _finger_bellow_geometry() -> MeshGeometry:
    """One connected corrugated soft actuator, authored in the finger frame.

    Local +X points away from the gripper center, local -Z points down the
    hanging soft finger. The alternating oval fins give the silicone bellow
    the image's ribbed pneumatic-actuator look while a hidden central web keeps
    the mesh a single supported body.
    """

    geom = MeshGeometry()

    # A compliant inner web tying all bellows into one continuous silicone part.
    geom.merge(BoxGeometry((0.022, 0.018, 0.082)).translate(0.020, 0.0, -0.088))

    # Stacked oval pressure chambers / fins.
    for i, z in enumerate([-0.050, -0.060, -0.071, -0.082, -0.093, -0.104, -0.115, -0.126]):
        radius = 0.0195 if i % 2 == 0 else 0.0175
        fin = CylinderGeometry(radius, 0.0075, radial_segments=32, closed=True)
        fin.scale(1.10, 0.58, 1.0).translate(0.021, 0.0, z)
        geom.merge(fin)

    # Soft rounded distal pad, slightly flatter on the grasping side.
    tip = SphereGeometry(0.017, width_segments=28, height_segments=12)
    tip.scale(1.00, 0.62, 0.70).translate(0.020, 0.0, -0.137)
    geom.merge(tip)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="soft_pneumatic_gripper",
        meta={
            "category": "Robotics / Soft pneumatic gripper",
            "reference_note": "Reference image matches the requested soft pneumatic gripper category.",
        },
    )

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_slot = model.material("black_rail_slots", rgba=(0.02, 0.025, 0.03, 1.0))
    soft_blue = model.material("matte_cyan_silicone", rgba=(0.04, 0.55, 0.70, 1.0))
    blue_shadow = model.material("darker_bellow_valleys", rgba=(0.02, 0.34, 0.46, 1.0))
    nylon = model.material("off_white_pneumatic_fittings", rgba=(0.88, 0.84, 0.75, 1.0))
    rubber = model.material("black_flexible_air_tube", rgba=(0.01, 0.012, 0.012, 1.0))

    manifold = model.part("manifold")

    # ------------------------------------------------------------------
    # Inline single-rail bank: one long manifold beam along +X
    # ------------------------------------------------------------------
    n_fingers = 4
    pitch = 0.065  # spacing between finger stations along the beam
    x0 = -((n_fingers - 1) / 2.0) * pitch  # first station X
    station_xs = [x0 + idx * pitch for idx in range(n_fingers)]
    finger_z = 0.130  # hinge-line height

    # Elongated mounting plate drilled for the inline beam layout.
    plate_holes = [_circle_profile(0.004, center=(0.0, 0.0))]
    for hx, hy in (
        (0.100, 0.022), (-0.100, 0.022),
        (0.100, -0.022), (-0.100, -0.022),
        (0.0, 0.022), (0.0, -0.022),
    ):
        plate_holes.append(_circle_profile(0.0032, center=(hx, hy)))
    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.260, 0.060, 0.012, corner_segments=8),
            plate_holes,
            0.008,
            center=True,
        ),
        "perforated_top_plate",
    )
    manifold.visual(top_plate_mesh, origin=Origin(xyz=(0.0, 0.0, 0.176)), material=aluminum, name="mounting_plate")

    # Central manifold boss rises through the beam to carry the mounting plate.
    manifold.visual(
        Cylinder(radius=0.024, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=aluminum,
        name="central_boss",
    )
    # Pneumatic distribution block sits above the beam around the boss.
    manifold.visual(
        Box((0.048, 0.036, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=aluminum,
        name="air_manifold_block",
    )

    # Single inline manifold beam along +X (replaces the crossed rails).
    manifold.visual(
        Box((0.260, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        material=aluminum,
        name="beam",
    )
    # Dark pneumatic channel on top of the beam.
    manifold.visual(
        Box((0.240, 0.006, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, 0.1496)),
        material=dark_slot,
        name="beam_slot",
    )

    # Plate screws and stand-off posts visibly fix the plate to the beam assembly.
    for i, (sx, sy) in enumerate(((0.100, 0.022), (-0.100, 0.022), (0.100, -0.022), (-0.100, -0.022))):
        manifold.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(sx, sy, 0.186)),
            material=aluminum,
            name=f"standoff_{i}",
        )
        manifold.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(sx, sy, 0.1935)),
            material=aluminum,
            name=f"cap_screw_{i}",
        )

    # ------------------------------------------------------------------
    # Per-station yokes, hinge pins, elbows, and air tubes (indexed loop)
    # ------------------------------------------------------------------
    for idx in range(n_fingers):
        sx = station_xs[idx]

        # Two fork cheeks straddle the moving actuator block but leave it clear.
        for side in (-1.0, 1.0):
            ly = side * 0.0235
            manifold.visual(
                Box((0.026, 0.006, 0.040)),
                origin=Origin(xyz=(sx, ly, 0.123)),
                material=aluminum,
                name=f"yoke_cheek_{idx}_{0 if side < 0 else 1}",
            )
        manifold.visual(
            Box((0.028, 0.054, 0.006)),
            origin=Origin(xyz=(sx, 0.0, 0.141)),
            material=aluminum,
            name=f"yoke_bridge_{idx}",
        )
        manifold.visual(
            Cylinder(radius=0.0042, length=0.055),
            origin=Origin(xyz=(sx, 0.0, finger_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"hinge_pin_{idx}",
        )

        # Pale pneumatic elbow on the beam beside each finger station.
        manifold.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(xyz=(sx, 0.020, 0.136), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=nylon,
            name=f"fixed_elbow_{idx}",
        )

    # Flexible hoses arc from the central area along the beam to each station.
    for idx in range(n_fingers):
        sx = station_xs[idx]
        mx = sx * 0.5
        tube = tube_from_spline_points(
            [
                (0.0, 0.0, 0.162),
                (mx, 0.0, 0.168),
                (sx, 0.0, 0.148),
            ],
            radius=0.0026,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        manifold.visual(
            mesh_from_geometry(tube, f"air_tube_{idx}"),
            material=rubber,
            name=f"air_tube_{idx}",
        )

    # ------------------------------------------------------------------
    # Articulated soft fingers and hose connectors (indexed loop)
    # ------------------------------------------------------------------
    for idx in range(n_fingers):
        sx = station_xs[idx]

        finger = model.part(f"finger_{idx}")
        # Actuator base centered on the hinge pin for clean inline spacing.
        finger.visual(
            Box((0.030, 0.034, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=aluminum,
            name="actuator_base",
        )
        finger.visual(
            Box((0.019, 0.021, 0.024)),
            origin=Origin(xyz=(0.006, 0.0, -0.041)),
            material=soft_blue,
            name="soft_neck",
        )
        finger.visual(
            mesh_from_geometry(_finger_bellow_geometry(), f"finger_bellow_{idx}"),
            origin=Origin(),
            material=soft_blue,
            name="bellow",
        )
        # Darker thin insets on the grasping face read as compressed valley texture.
        for j, z in enumerate((-0.058, -0.080, -0.102, -0.124)):
            finger.visual(
                Box((0.004, 0.016, 0.003)),
                origin=Origin(xyz=(0.0015, 0.0, z)),
                material=blue_shadow,
                name=f"grip_valley_{j}",
            )

        # Revolute joint: axis perpendicular to the beam (Y), fingers curl in -X direction.
        model.articulation(
            f"manifold_to_finger_{idx}",
            ArticulationType.REVOLUTE,
            parent=manifold,
            child=finger,
            origin=Origin(xyz=(sx, 0.0, finger_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=0.62),
        )

        # A small swiveling hose connector on the side of each actuator block.
        # Shortened reach to clear adjacent stations in the inline layout.
        connector = model.part(f"hose_connector_{idx}")
        connector.visual(
            Cylinder(radius=0.0052, length=0.022),
            origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nylon,
            name="barbed_port",
        )
        connector.visual(
            Cylinder(radius=0.0068, length=0.004),
            origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nylon,
            name="port_collar",
        )
        stub = tube_from_spline_points(
            [
                (0.022, 0.0, 0.0),
                (0.026, 0.002, 0.004),
                (0.028, 0.003, 0.010),
            ],
            radius=0.0024,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
        connector.visual(
            mesh_from_geometry(stub, f"hose_stub_{idx}"),
            material=rubber,
            name="hose_stub",
        )
        model.articulation(
            f"finger_to_connector_{idx}",
            ArticulationType.REVOLUTE,
            parent=finger,
            child=connector,
            origin=Origin(xyz=(0.015, 0.0, -0.018)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=1.2, lower=-0.30, upper=0.30),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    manifold = object_model.get_part("manifold")

    ctx.check(
        "four flexible fingers are modeled",
        all(object_model.get_part(f"finger_{i}") is not None for i in range(4)),
    )
    ctx.check(
        "four swiveling hose connectors are modeled",
        all(object_model.get_part(f"hose_connector_{i}") is not None for i in range(4)),
    )

    # ---- Inline beam arrangement (structural delta assertion) ----
    joint_origins = []
    for i in range(4):
        j = object_model.get_articulation(f"manifold_to_finger_{i}")
        joint_origins.append(j.origin.xyz)

    ys = [o[1] for o in joint_origins]
    zs = [o[2] for o in joint_origins]
    xs = sorted(o[0] for o in joint_origins)
    ctx.check(
        "inline beam: all finger joints share Y=0",
        all(abs(y) < 1e-6 for y in ys),
        details=f"ys={ys}",
    )
    ctx.check(
        "inline beam: all finger joints share the same Z height",
        all(abs(z - zs[0]) < 1e-6 for z in zs),
        details=f"zs={zs}",
    )
    ctx.check(
        "inline beam: four distinct X stations",
        len(set(round(x, 6) for x in xs)) == 4,
        details=f"xs={xs}",
    )
    # Even spacing: consecutive X gaps should be approximately equal.
    gaps = [xs[i + 1] - xs[i] for i in range(len(xs) - 1)]
    if len(gaps) >= 2:
        ctx.check(
            "inline beam: finger stations are evenly spaced along X",
            max(abs(g - gaps[0]) for g in gaps) < 0.002,
            details=f"gaps={gaps}",
        )
    # Beam visual must exist on the manifold.
    ctx.check(
        "manifold has inline beam visual",
        manifold.get_visual("beam") is not None,
    )

    # ---- Per-station hinge, clearance, bend, and swivel checks ----
    for i in range(4):
        finger = object_model.get_part(f"finger_{i}")
        joint = object_model.get_articulation(f"manifold_to_finger_{i}")
        connector_joint = object_model.get_articulation(f"finger_to_connector_{i}")

        ctx.allow_overlap(
            manifold,
            finger,
            elem_a=f"hinge_pin_{i}",
            elem_b="actuator_base",
            reason="The metal hinge pin is intentionally captured through the actuator block bore.",
        )
        ctx.expect_overlap(
            manifold,
            finger,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b="actuator_base",
            min_overlap=0.0035,
            name=f"finger {i} hinge pin passes through block height",
        )
        ctx.expect_overlap(
            finger,
            manifold,
            axes="xy",
            elem_a="actuator_base",
            elem_b=f"hinge_pin_{i}",
            min_overlap=0.004,
            name=f"finger {i} hinge pin is captured in actuator bore",
        )
        ctx.expect_gap(
            manifold,
            finger,
            axis="z",
            positive_elem="beam",
            negative_elem="actuator_base",
            min_gap=0.001,
            max_gap=0.025,
            name=f"finger {i} actuator hangs below beam",
        )

        # Positive bend curls each soft bellow in -X (right-hand rule around +Y axis).
        def _element_center_xyz(part, elem):
            aabb = ctx.part_element_world_aabb(part, elem=elem)
            if aabb is None:
                return None
            lo, hi = aabb
            return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

        rest_center = _element_center_xyz(finger, "bellow")
        with ctx.pose({joint: 0.55}):
            bent_center = _element_center_xyz(finger, "bellow")
        if rest_center is not None and bent_center is not None:
            ctx.check(
                f"finger {i} bellow curls under positive pressure",
                bent_center[0] < rest_center[0] - 0.020,
                details=f"rest_x={rest_center[0]:.4f}, bent_x={bent_center[0]:.4f}",
            )
        else:
            ctx.fail(f"finger {i} bellow center measurable", "Missing bellow element AABB")

        connector = object_model.get_part(f"hose_connector_{i}")
        rest_conn = ctx.part_world_aabb(connector)
        with ctx.pose({connector_joint: 0.25}):
            swivel_conn = ctx.part_world_aabb(connector)
        if rest_conn is not None and swivel_conn is not None:
            rest_cz = (rest_conn[0][2] + rest_conn[1][2]) / 2.0
            swivel_cz = (swivel_conn[0][2] + swivel_conn[1][2]) / 2.0
            ctx.check(
                f"hose connector {i} has visible swivel travel",
                abs(swivel_cz - rest_cz) > 0.0005,
                details=f"rest_z={rest_cz:.4f}, swivel_z={swivel_cz:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
