from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geom in geometries:
        merged.merge(geom)
    return merged


def _curved_grip_panel(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    angle_min: float,
    angle_max: float,
    segments: int = 18,
) -> MeshGeometry:
    """A shallow raised, curved rectangular pad on the cylindrical pump body."""

    geom = MeshGeometry()
    inner_bottom: list[int] = []
    outer_bottom: list[int] = []
    inner_top: list[int] = []
    outer_top: list[int] = []

    for i in range(segments + 1):
        t = angle_min + (angle_max - angle_min) * i / segments
        sin_t = math.sin(t)
        cos_t = math.cos(t)
        inner_bottom.append(geom.add_vertex(inner_radius * sin_t, inner_radius * cos_t, z_min))
        inner_top.append(geom.add_vertex(inner_radius * sin_t, inner_radius * cos_t, z_max))
        outer_bottom.append(geom.add_vertex(outer_radius * sin_t, outer_radius * cos_t, z_min))
        outer_top.append(geom.add_vertex(outer_radius * sin_t, outer_radius * cos_t, z_max))

    for i in range(segments):
        # Front curved surface.
        geom.add_face(outer_bottom[i], outer_bottom[i + 1], outer_top[i + 1])
        geom.add_face(outer_bottom[i], outer_top[i + 1], outer_top[i])
        # Back surface seated slightly into the green shell.
        geom.add_face(inner_bottom[i + 1], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[i + 1], inner_top[i], inner_top[i + 1])
        # Top and bottom narrow faces.
        geom.add_face(inner_top[i], outer_top[i], outer_top[i + 1])
        geom.add_face(inner_top[i], outer_top[i + 1], inner_top[i + 1])
        geom.add_face(inner_bottom[i + 1], outer_bottom[i + 1], outer_bottom[i])
        geom.add_face(inner_bottom[i + 1], outer_bottom[i], inner_bottom[i])

    # End caps.
    geom.add_face(inner_bottom[0], outer_bottom[0], outer_top[0])
    geom.add_face(inner_bottom[0], outer_top[0], inner_top[0])
    geom.add_face(outer_bottom[-1], inner_bottom[-1], inner_top[-1])
    geom.add_face(outer_bottom[-1], inner_top[-1], outer_top[-1])
    return geom


def _finger_scallop(x: float, z: float) -> MeshGeometry:
    """Small olive raised bars that cut into the black grip silhouette."""

    return (
        CylinderGeometry(radius=0.0048, height=0.030, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(x, 0.0495, z)
    )


def _pump_base_geom() -> MeshGeometry:
    """Combined threaded bottle-neck collar and transition shroud as one solid lathe.

    The lower section has external thread ridges for bottle engagement; the upper
    section tapers outward to meet the main cylindrical pump body.
    """

    profile = [
        (0.016, 0.000),
        (0.019, 0.002),
        (0.016, 0.004),
        (0.019, 0.006),
        (0.016, 0.008),
        (0.019, 0.010),
        (0.016, 0.012),
        (0.019, 0.014),
        (0.016, 0.016),
        (0.019, 0.018),
        (0.016, 0.020),
        (0.019, 0.022),
        (0.017, 0.024),
        (0.019, 0.027),
        (0.028, 0.032),
        (0.038, 0.037),
        (0.044, 0.041),
    ]
    return LatheGeometry(profile, segments=48, closed=True)


def _bottle_shell(height: float = 0.210) -> MeshGeometry:
    """Thin-walled transparent PET-style water bottle with neck and shoulder."""

    outer = [
        (0.000, 0.000),
        (0.028, 0.000),
        (0.032, 0.003),
        (0.034, 0.012),
        (0.034, 0.152),
        (0.032, 0.162),
        (0.018, 0.178),
        (0.017, 0.185),
        (0.017, 0.198),
        (0.018, height),
    ]
    inner = [
        (0.000, 0.002),
        (0.026, 0.002),
        (0.030, 0.005),
        (0.032, 0.014),
        (0.032, 0.150),
        (0.030, 0.160),
        (0.016, 0.176),
        (0.014, 0.183),
        (0.014, 0.198),
        (0.015, height),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=56,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )


def _hose_from_world_points(
    points: list[tuple[float, float, float]],
    origin_xyz: tuple[float, float, float],
    *,
    radius: float,
) -> MeshGeometry:
    local_points = [
        (x - origin_xyz[0], y - origin_xyz[1], z - origin_xyz[2]) for x, y, z in points
    ]
    return tube_from_spline_points(
        local_points,
        radius=radius,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="bottle_top_water_filter_pump",
        meta={
            "run_notes": (
                "Bottle-top mounted hand pump filter variant: the free-standing base is "
                "replaced with a threaded bottle-neck collar and a clear mounted bottle "
                "below. Clean filtered water discharges directly into the screwed-on "
                "bottle. The intake side remains for drawing from a dirty water source."
            )
        },
    )

    olive = model.material("olive_green_plastic", rgba=(0.15, 0.27, 0.055, 1.0))
    dark_olive = model.material("dark_olive_shadow", rgba=(0.08, 0.16, 0.035, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    clear_tube = model.material("clear_flexible_tube", rgba=(0.86, 0.93, 1.0, 0.42))
    smoky_tube = model.material("smoky_intake_tube", rgba=(0.75, 0.58, 0.48, 0.48))
    clear_bottle = model.material("clear_bottle_plastic", rgba=(0.88, 0.93, 1.0, 0.28))
    clear_water = model.material("clear_water", rgba=(0.72, 0.86, 1.0, 0.24))

    # Root: the pump body carries the top manifold, plunger sleeve, hose sockets,
    # and the threaded bottle-neck collar at the base.
    body = model.part("pump_body")
    body.visual(
        Cylinder(radius=0.045, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=olive,
        name="cylindrical_body",
    )
    # Threaded bottle-neck collar with integrated transition shroud
    # (replaces rounded_bottom_cap). Thread ridges below, taper above to body.
    body.visual(
        mesh_from_geometry(_pump_base_geom(), "bottle_thread_collar"),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_olive,
        name="bottle_thread_collar",
    )
    body.visual(
        Cylinder(radius=0.0475, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=olive,
        name="top_cap",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(-0.018, 0.0, 0.245)),
        material=dark_olive,
        name="top_manifold_disk",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.052, 0.0, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="outlet_socket",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.052, 0.0, 0.235), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="intake_socket",
    )
    sleeve_geom = LatheGeometry.from_shell_profiles(
        [(0.017, 0.000), (0.021, 0.004), (0.022, 0.040), (0.020, 0.050)],
        [(0.0125, 0.002), (0.0135, 0.008), (0.0135, 0.046)],
        segments=48,
        start_cap="flat",
        end_cap="round",
        lip_samples=6,
    )
    body.visual(
        mesh_from_geometry(sleeve_geom, "plunger_sleeve"),
        origin=Origin(xyz=(0.020, 0.0, 0.240)),
        material=olive,
        name="plunger_sleeve",
    )
    body.visual(
        mesh_from_geometry(
            _curved_grip_panel(
                inner_radius=0.0435,
                outer_radius=0.0515,
                z_min=0.055,
                z_max=0.172,
                angle_min=math.radians(-42),
                angle_max=math.radians(42),
            ),
            "black_grip_panel",
        ),
        material=black_rubber,
        name="black_grip_panel",
    )
    grip_marks = _merge(
        [_finger_scallop(0.038, z) for z in (0.070, 0.090, 0.110, 0.130, 0.150)]
    )
    body.visual(
        mesh_from_geometry(grip_marks, "green_grip_reliefs"),
        material=olive,
        name="green_grip_reliefs",
    )

    # Sliding plunger: rod is centered in the hollow sleeve and has a T-style hand grip.
    plunger = model.part("plunger_handle")
    plunger.visual(
        Cylinder(radius=0.010, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=olive,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        material=olive,
        name="upper_rod_collar",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="t_handle_grip",
    )
    plunger.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=olive,
        name="handle_neck",
    )

    plunger_joint = model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.020, 0.0, 0.258)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.075),
    )

    # Mounted bottle: clear PET bottle that screws into the threaded collar below the pump.
    # Clean filtered water discharges directly into this bottle.
    bottle = model.part("mounted_bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_shell(0.210), "bottle_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=clear_bottle,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.033, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=clear_water,
        name="clean_water_level",
    )

    model.articulation(
        "body_to_bottle",
        ArticulationType.FIXED,
        parent=body,
        child=bottle,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )

    # Detachable intake port: draws dirty water from an external source through a hose.
    intake_origin = (0.057, 0.0, 0.235)
    intake_port = model.part("intake_port")
    intake_port.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=olive,
        name="intake_barb",
    )
    intake_port.visual(
        Cylinder(radius=0.0105, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_olive,
        name="intake_collar",
    )
    intake_tube = _hose_from_world_points(
        [
            (0.087, 0.0, 0.235),
            (0.130, 0.0, 0.300),
            (0.180, 0.0, 0.330),
            (0.220, 0.0, 0.250),
            (0.190, 0.0, 0.060),
        ],
        intake_origin,
        radius=0.0046,
    )
    intake_port.visual(
        mesh_from_geometry(intake_tube, "smoky_intake_tube"),
        material=smoky_tube,
        name="smoky_intake_tube",
    )
    intake_port.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.133, 0.0, -0.179)),
        material=black_rubber,
        name="black_sample_cap",
    )
    intake_port.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(
            xyz=(0.138, 0.0, -0.205),
            rpy=(0.18, 0.0, 0.0),
        ),
        material=smoky_tube,
        name="short_filter_straw",
    )

    intake_joint = model.articulation(
        "body_to_intake_port",
        ArticulationType.PRISMATIC,
        parent=body,
        child=intake_port,
        origin=Origin(xyz=intake_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.030),
    )

    # Store names for tests without relying on local variables.
    model.meta["primary_joints"] = (
        plunger_joint.name,
        intake_joint.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("pump_body")
    plunger = object_model.get_part("plunger_handle")
    intake_port = object_model.get_part("intake_port")
    mounted_bottle = object_model.get_part("mounted_bottle")
    plunger_joint = object_model.get_articulation("body_to_plunger")
    intake_joint = object_model.get_articulation("body_to_intake_port")

    # Plunger rod passes through the simplified top cap and into the pump cylinder.
    ctx.allow_overlap(
        plunger,
        body,
        elem_a="plunger_rod",
        elem_b="top_cap",
        reason=(
            "The plunger rod intentionally passes through the simplified molded top cap "
            "as a captured shaft guide."
        ),
    )
    ctx.allow_overlap(
        plunger,
        body,
        elem_a="plunger_rod",
        elem_b="cylindrical_body",
        reason=(
            "The lower plunger rod is intentionally retained inside the simplified "
            "solid pump-cylinder proxy, representing the hidden piston stroke."
        ),
    )
    # Bottle neck engages the threaded collar (intentional threaded fit).
    ctx.allow_overlap(
        mounted_bottle,
        body,
        elem_a="bottle_shell",
        elem_b="bottle_thread_collar",
        reason=(
            "The bottle neck intentionally inserts into the threaded collar shell, "
            "representing the screw-on bottle engagement."
        ),
    )

    ctx.check(
        "reference-critical parts exist",
        all(p is not None for p in (body, plunger, intake_port, mounted_bottle)),
        details="Pump body, plunger, intake port, and mounted bottle should be authored.",
    )

    # Structural axis assertion: bottle_thread_collar exists on the pump body
    # as the bottle mating interface replacing the free-standing bottom cap.
    collar_visual = body.get_visual("bottle_thread_collar")
    ctx.check(
        "bottle_thread_collar is the pump base interface",
        collar_visual is not None,
        details="Threaded bottle-neck collar must exist on pump_body as the bottle mating interface.",
    )

    # The mounted bottle hangs below the pump body via the FIXED body_to_bottle joint.
    # Prove the bottle shell top is below the pump cylinder bottom.
    ctx.expect_gap(
        body,
        mounted_bottle,
        axis="z",
        positive_elem="cylindrical_body",
        negative_elem="bottle_shell",
        min_gap=0.010,
        name="bottle shell hangs below pump cylinder",
    )

    # Bottle neck engages the threaded collar (Z overlap proves insertion).
    ctx.expect_overlap(
        mounted_bottle,
        body,
        axes="z",
        elem_a="bottle_shell",
        elem_b="bottle_thread_collar",
        min_overlap=0.008,
        name="bottle neck inserted into threaded collar",
    )

    # Plunger retains in sleeve at rest.
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="plunger_sleeve",
        margin=0.0015,
        name="plunger rod stays centered in sleeve",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="plunger_sleeve",
        min_overlap=0.025,
        name="plunger remains inserted in sleeve at rest",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="top_cap",
        margin=0.002,
        name="plunger rod passes through top cap guide",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="top_cap",
        min_overlap=0.020,
        name="top cap captures plunger rod",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_rod",
        outer_elem="cylindrical_body",
        margin=0.002,
        name="hidden plunger rod centered in cylinder",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_rod",
        elem_b="cylindrical_body",
        min_overlap=0.030,
        name="hidden plunger rod remains inside cylinder",
    )

    # Intake port seats on its socket.
    ctx.expect_contact(
        intake_port,
        body,
        elem_a="intake_collar",
        elem_b="intake_socket",
        contact_tol=0.001,
        name="intake port seated on socket",
    )

    plunger_rest = ctx.part_world_position(plunger)
    intake_rest = ctx.part_world_position(intake_port)

    with ctx.pose({plunger_joint: 0.075, intake_joint: 0.030}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="plunger_rod",
            outer_elem="plunger_sleeve",
            margin=0.0015,
            name="raised plunger remains coaxial",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_rod",
            elem_b="plunger_sleeve",
            min_overlap=0.020,
            name="raised plunger still retained",
        )
        plunger_raised = ctx.part_world_position(plunger)
        intake_detached = ctx.part_world_position(intake_port)

    ctx.check(
        "plunger handle slides upward",
        plunger_rest is not None
        and plunger_raised is not None
        and plunger_raised[2] > plunger_rest[2] + 0.060,
        details=f"rest={plunger_rest}, raised={plunger_raised}",
    )
    ctx.check(
        "intake port detaches outward",
        intake_rest is not None
        and intake_detached is not None
        and intake_detached[0] > intake_rest[0] + 0.020,
        details=f"rest={intake_rest}, detached={intake_detached}",
    )

    return ctx.report()


object_model = build_object_model()
