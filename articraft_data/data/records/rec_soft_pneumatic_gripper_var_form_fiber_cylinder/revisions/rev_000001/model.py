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
    """Smooth tapered fiber-reinforced soft actuator body.

    Local +X points away from the gripper center, local -Z points down the
    hanging soft finger. The body is a smooth slightly-tapered cylinder
    (wider at base, narrower at tip) representing a PneuFlex / McKibben-style
    silicone actuator that curls inward under pressure.
    """

    geom = MeshGeometry()

    # Smooth tapered cylinder body built from thin overlapping slices.
    z_top = -0.048
    z_bot = -0.128
    r_top = 0.018
    r_bot = 0.013
    n_slices = 12
    span = z_top - z_bot  # 0.080 m
    slice_h = span / n_slices

    for i in range(n_slices):
        t = (i + 0.5) / n_slices
        r = r_top + t * (r_bot - r_top)
        z = z_top - (i + 0.5) * slice_h
        sl = CylinderGeometry(r, slice_h * 1.05, radial_segments=32, closed=True)
        sl.translate(0.020, 0.0, z)
        geom.merge(sl)

    # Soft rounded distal pad at the tip.
    tip = SphereGeometry(0.014, width_segments=24, height_segments=12)
    tip.scale(1.0, 0.80, 0.75).translate(0.020, 0.0, -0.135)
    geom.merge(tip)

    return geom


def _fiber_wrap_mesh(idx: int):
    """Helical fiber-wrap tube for one finger actuator.

    Creates a thin tube following a spiral path around the tapered actuator
    body, representing the Kevlar / polyester fiber reinforcement that
    constrains radial expansion and converts pressure into bending.
    """
    z_top = -0.050
    z_bot = -0.126
    r_top = 0.019  # slightly proud of body surface
    r_bot = 0.014
    n_turns = 2.5
    pts_per_turn = 14
    n_pts = int(n_turns * pts_per_turn) + 1

    points = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        z = z_top + t * (z_bot - z_top)
        r = r_top + t * (r_bot - r_top)
        angle = t * n_turns * 2.0 * math.pi
        x = 0.020 + r * math.cos(angle)
        y = r * math.sin(angle)
        points.append((x, y, z))

    tube = tube_from_spline_points(
        points,
        radius=0.0012,
        samples_per_segment=10,
        radial_segments=8,
        cap_ends=True,
    )
    return mesh_from_geometry(tube, f"fiber_wrap_{idx}")


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
    fiber_mat = model.material("dark_fiber_wrap", rgba=(0.12, 0.13, 0.16, 1.0))
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

    # Root-side yokes and hinge pins at the four actuator stations.
    finger_radius = 0.101
    finger_z = 0.130
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
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

    # Four flexible hoses arc from the central boss toward the actuator stations.
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
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

    # Four articulated soft fingers, each with a metal actuator block, soft neck, and fiber-reinforced tapered actuator.
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
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
        # Helical fiber-wrap rib visible on the actuator surface.
        finger.visual(
            _fiber_wrap_mesh(idx),
            origin=Origin(),
            material=fiber_mat,
            name="fiber_wrap",
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
            origin=Origin(xyz=(0.0265, 0.0, -0.018)),
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

    ctx.check("four flexible fingers are modeled", all(object_model.get_part(f"finger_{i}") is not None for i in range(4)))
    ctx.check("four swiveling hose connectors are modeled", all(object_model.get_part(f"hose_connector_{i}") is not None for i in range(4)))

    # Verify each finger has a smooth tapered cylindrical actuator (not oval/boxy corrugated fins)
    # and the helical fiber-wrap surface feature.
    for i in range(4):
        finger_part = object_model.get_part(f"finger_{i}")
        bellow_vis = finger_part.get_visual("bellow") if finger_part else None
        fiber_vis = finger_part.get_visual("fiber_wrap") if finger_part else None
        ctx.check(
            f"finger_{i} has smooth tapered actuator bellow",
            bellow_vis is not None,
            details=f"bellow visual missing on finger_{i}",
        )
        ctx.check(
            f"finger_{i} has helical fiber_wrap surface feature",
            fiber_vis is not None,
            details=f"fiber_wrap visual missing on finger_{i}",
        )
    # Check the bellow on finger_0 reads as cylindrical (dx ≈ dy) not oval/boxy.
    f0 = object_model.get_part("finger_0")
    f0_bellow_aabb = ctx.part_element_world_aabb(f0, elem="bellow")
    if f0_bellow_aabb is not None:
        lo, hi = f0_bellow_aabb
        dx = hi[0] - lo[0]
        dy = hi[1] - lo[1]
        dz = hi[2] - lo[2]
        ratio = min(dx, dy) / max(dx, dy) if max(dx, dy) > 1e-6 else 0.0
        ctx.check(
            "finger_0 bellow is cylindrical (dx/dy ≈ 1, not oval fins)",
            ratio > 0.80,
            details=f"dx={dx:.4f}, dy={dy:.4f}, ratio={ratio:.3f}",
        )
        ctx.check(
            "finger_0 bellow is taller than wide (elongated actuator)",
            dz > max(dx, dy) * 1.5,
            details=f"dz={dz:.4f}, max_xy={max(dx, dy):.4f}",
        )

    # Hinge pins are intentionally captured in the actuator base to show a real pivot.
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
            positive_elem="rail_x" if i in (0, 2) else "rail_y",
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

        connector = object_model.get_part(f"hose_connector_{i}")
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
