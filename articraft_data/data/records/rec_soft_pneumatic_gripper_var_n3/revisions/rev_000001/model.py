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

    # Four-hole rounded mounting plate, like the drilled metal top plate in the reference.
    plate_holes = [_circle_profile(0.004, center=(0.0, 0.0))]
    for hx, hy in ((0.024, 0.018), (-0.024, 0.018), (0.024, -0.018), (-0.024, -0.018), (0.0, 0.028), (0.0, -0.028)):
        plate_holes.append(_circle_profile(0.0032, center=(hx, hy)))
    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.098, 0.074, 0.013, corner_segments=8),
            plate_holes,
            0.008,
            center=True,
        ),
        "perforated_top_plate",
    )
    manifold.visual(top_plate_mesh, origin=Origin(xyz=(0.0, 0.0, 0.176)), material=aluminum, name="mounting_plate")

    # Central manifold boss and a lower pneumatic block carry the arms and route air.
    manifold.visual(Cylinder(radius=0.024, length=0.044), origin=Origin(xyz=(0.0, 0.0, 0.150)), material=aluminum, name="central_boss")
    manifold.visual(Box((0.050, 0.050, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.128)), material=aluminum, name="air_manifold_block")

    # Crossed slotted rails: aluminum bars with black channels on their top faces.
    manifold.visual(Box((0.230, 0.021, 0.008)), origin=Origin(xyz=(0.0, 0.0, 0.144)), material=aluminum, name="rail_x")
    manifold.visual(Box((0.021, 0.230, 0.008)), origin=Origin(xyz=(0.0, 0.0, 0.144)), material=aluminum, name="rail_y")
    manifold.visual(Box((0.205, 0.006, 0.0022)), origin=Origin(xyz=(0.0, 0.0, 0.1491)), material=dark_slot, name="slot_x")
    manifold.visual(Box((0.006, 0.205, 0.0022)), origin=Origin(xyz=(0.0, 0.0, 0.1491)), material=dark_slot, name="slot_y")

    # Plate screws and stand-off posts visibly fix the manifold to the rail assembly.
    for i, (sx, sy) in enumerate(((0.030, 0.020), (-0.030, 0.020), (0.030, -0.020), (-0.030, -0.020))):
        manifold.visual(Cylinder(radius=0.004, length=0.012), origin=Origin(xyz=(sx, sy, 0.186)), material=aluminum, name=f"standoff_{i}")
        manifold.visual(Cylinder(radius=0.006, length=0.003), origin=Origin(xyz=(sx, sy, 0.1935)), material=aluminum, name=f"cap_screw_{i}")

    # Root-side yokes and hinge pins at the three actuator stations.
    finger_radius = 0.101
    finger_z = 0.130
    _finger_yaws = tuple(2.0 * math.pi * i / 3.0 for i in range(3))

    # Radial support arms bridge the central manifold to each finger station.
    for idx, yaw in enumerate(_finger_yaws):
        arm_inner = 0.028
        arm_outer = finger_radius - 0.014
        mid_r = (arm_inner + arm_outer) / 2.0
        arm_len = arm_outer - arm_inner
        mx, my = _rot_xy(mid_r, yaw)
        manifold.visual(
            Box((arm_len, 0.016, 0.008)),
            origin=Origin(xyz=(mx, my, 0.144), rpy=(0.0, 0.0, yaw)),
            material=aluminum,
            name=f"radial_arm_{idx}",
        )

    for idx, yaw in enumerate(_finger_yaws):
        x, y = _rot_xy(finger_radius, yaw)
        # Two fork cheeks straddle the moving actuator block but leave it clear.
        for side in (-1.0, 1.0):
            ly = side * 0.0235
            wx = x + ly * math.cos(yaw + math.pi / 2.0)
            wy = y + ly * math.sin(yaw + math.pi / 2.0)
            manifold.visual(
                Box((0.026, 0.006, 0.040)),
                origin=Origin(xyz=(wx, wy, 0.123), rpy=(0.0, 0.0, yaw)),
                material=aluminum,
                name=f"yoke_cheek_{idx}_{0 if side < 0 else 1}",
            )
        manifold.visual(
            Box((0.028, 0.054, 0.006)),
            origin=Origin(xyz=(x, y, 0.141), rpy=(0.0, 0.0, yaw)),
            material=aluminum,
            name=f"yoke_bridge_{idx}",
        )
        manifold.visual(
            Cylinder(radius=0.0042, length=0.055),
            origin=Origin(xyz=(x, y, finger_z), rpy=(math.pi / 2.0, 0.0, yaw)),
            material=aluminum,
            name=f"hinge_pin_{idx}",
        )
        # Pale pneumatic elbow on the rail beside each finger.
        fx, fy = _rot_xy(0.083, yaw)
        manifold.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(xyz=(fx, fy, 0.136), rpy=(math.pi / 2.0, 0.0, yaw)),
            material=nylon,
            name=f"fixed_elbow_{idx}",
        )

    # Three flexible hoses arc from the central boss toward the actuator stations.
    for idx, yaw in enumerate(_finger_yaws):
        sx, sy = _rot_xy(0.020, yaw)
        mx, my = _rot_xy(0.060, yaw)
        ex, ey = _rot_xy(0.086, yaw)
        tube = tube_from_spline_points(
            [
                (sx, sy, 0.158),
                (mx, my, 0.164),
                (ex, ey, 0.145),
            ],
            radius=0.0026,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        manifold.visual(mesh_from_geometry(tube, f"air_tube_{idx}"), material=rubber, name=f"air_tube_{idx}")

    # Three articulated soft fingers, each with a metal actuator block, neck, and corrugated silicone bellow.
    for idx, yaw in enumerate(_finger_yaws):
        finger = model.part(f"finger_{idx}")
        finger.visual(
            Box((0.030, 0.034, 0.034)),
            origin=Origin(xyz=(0.012, 0.0, -0.017)),
            material=aluminum,
            name="actuator_base",
        )
        finger.visual(
            Box((0.019, 0.021, 0.024)),
            origin=Origin(xyz=(0.018, 0.0, -0.041)),
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

        jx, jy = _rot_xy(finger_radius, yaw)
        model.articulation(
            f"manifold_to_finger_{idx}",
            ArticulationType.REVOLUTE,
            parent=manifold,
            child=finger,
            origin=Origin(xyz=(jx, jy, finger_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=0.62),
        )

        # A small swiveling hose connector on the side of each actuator block.
        connector = model.part(f"hose_connector_{idx}")
        connector.visual(
            Cylinder(radius=0.0052, length=0.027),
            origin=Origin(xyz=(0.0135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nylon,
            name="barbed_port",
        )
        connector.visual(
            Cylinder(radius=0.0068, length=0.004),
            origin=Origin(xyz=(0.0022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nylon,
            name="port_collar",
        )
        stub = tube_from_spline_points(
            [
                (0.026, 0.0, 0.0),
                (0.041, 0.002, 0.006),
                (0.052, 0.006, 0.018),
            ],
            radius=0.0024,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        )
        connector.visual(mesh_from_geometry(stub, f"hose_stub_{idx}"), material=rubber, name="hose_stub")
        model.articulation(
            f"finger_to_connector_{idx}",
            ArticulationType.REVOLUTE,
            parent=finger,
            child=connector,
            origin=Origin(xyz=(0.027, 0.0, -0.018)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=1.2, lower=-0.30, upper=0.30),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    manifold = object_model.get_part("manifold")

    ctx.check("three flexible fingers are modeled", all(object_model.get_part(f"finger_{i}") is not None for i in range(3)))
    ctx.check("three swiveling hose connectors are modeled", all(object_model.get_part(f"hose_connector_{i}") is not None for i in range(3)))
    all_part_names = {p.name for p in object_model.parts}
    ctx.check("finger_3 does NOT exist (multiplicity reduced to 3)", "finger_3" not in all_part_names)

    # Verify the 3 fingers are evenly spaced at 120° (tripod layout).
    finger_positions = []
    for i in range(3):
        pos = ctx.part_world_position(object_model.get_part(f"finger_{i}"))
        if pos is not None:
            finger_positions.append(pos)
    if len(finger_positions) == 3:
        angles = [math.atan2(p[1], p[0]) for p in finger_positions]
        # Sort angles to compute consecutive gaps.
        angles_sorted = sorted(angles)
        gaps = [
            (angles_sorted[1] - angles_sorted[0]) % (2.0 * math.pi),
            (angles_sorted[2] - angles_sorted[1]) % (2.0 * math.pi),
            (angles_sorted[0] - angles_sorted[2]) % (2.0 * math.pi),
        ]
        target_gap = 2.0 * math.pi / 3.0
        max_angular_error = max(abs(g - target_gap) for g in gaps)
        ctx.check(
            "three finger stations are evenly spaced at 120° (tripod multiplicity)",
            max_angular_error < 0.15,
            details=f"angular gaps={[f'{g:.3f}' for g in gaps]}, max_error={max_angular_error:.3f} rad",
        )

    # Hinge pins are intentionally captured in the actuator base to show a real pivot.
    for i in range(3):
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
        connector = object_model.get_part(f"hose_connector_{i}")
        ctx.allow_overlap(
            finger,
            connector,
            elem_a="actuator_base",
            elem_b="barbed_port",
            reason="The barbed pneumatic port sits flush against the actuator block face; small mesh penetration at the coplanar mounting face is a tessellation artifact.",
        )
        ctx.expect_contact(
            finger,
            connector,
            elem_a="actuator_base",
            elem_b="barbed_port",
            contact_tol=0.002,
            name=f"finger {i} barbed port is mounted on actuator face",
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
            positive_elem="rail_x",
            negative_elem="actuator_base",
            min_gap=0.003,
            max_gap=0.020,
            name=f"finger {i} actuator hangs below support rail",
        )

        # Positive closure should bend each soft bellow inward toward the center.
        def _element_center_zxy(part, elem):
            aabb = ctx.part_element_world_aabb(part, elem=elem)
            if aabb is None:
                return None
            lo, hi = aabb
            return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

        rest_center = _element_center_zxy(finger, "bellow")
        with ctx.pose({joint: 0.55}):
            bent_center = _element_center_zxy(finger, "bellow")
        if rest_center is not None and bent_center is not None:
            rest_r = math.hypot(rest_center[0], rest_center[1])
            bent_r = math.hypot(bent_center[0], bent_center[1])
            ctx.check(
                f"finger {i} closes inward under positive pressure",
                bent_r < rest_r - 0.012,
                details=f"rest_radius={rest_r:.4f}, bent_radius={bent_r:.4f}",
            )
        else:
            ctx.fail(f"finger {i} bellow center measurable", "Missing bellow element AABB")

        rest_conn = ctx.part_world_aabb(connector)
        with ctx.pose({connector_joint: 0.25}):
            swivel_conn = ctx.part_world_aabb(connector)
        if rest_conn is not None and swivel_conn is not None:
            rest_cz = (rest_conn[0][2] + rest_conn[1][2]) / 2.0
            swivel_cz = (swivel_conn[0][2] + swivel_conn[1][2]) / 2.0
            ctx.check(
                f"hose connector {i} has visible swivel travel",
                abs(swivel_cz - rest_cz) > 0.001,
                details=f"rest_z={rest_cz:.4f}, swivel_z={swivel_cz:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
