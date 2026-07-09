from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


FABRIC = Material("warm_cream_fabric", rgba=(0.78, 0.70, 0.55, 1.0))
SEAM = Material("raised_seam_tape", rgba=(0.47, 0.39, 0.27, 1.0))
DARK_METAL = Material("satin_black_metal", rgba=(0.01, 0.011, 0.012, 1.0))
WORN_WOOD = Material("dark_jointed_wood", rgba=(0.22, 0.14, 0.08, 1.0))
TRAY_MAT = Material("matte_black_tray", rgba=(0.0, 0.0, 0.0, 1.0))


def _cylinder_rpy_for_xy(angle: float) -> tuple[float, float, float]:
    """Rotate a cylinder's local +Z axis into the XY direction at `angle`."""

    return (0.0, math.pi / 2.0, angle)


def _radius_at(z: float, axis: str) -> float:
    stations = [
        (0.12, 0.130, 0.072),
        (0.20, 0.145, 0.078),
        (0.34, 0.105, 0.070),
        (0.50, 0.158, 0.095),
        (0.63, 0.195, 0.104),
        (0.74, 0.250, 0.088),
        (0.82, 0.120, 0.062),
        (0.90, 0.062, 0.052),
    ]
    idx = 1 if axis == "x" else 2
    if z <= stations[0][0]:
        return stations[0][idx]
    for i in range(len(stations) - 1):
        z0, rx0, ry0 = stations[i]
        z1, rx1, ry1 = stations[i + 1]
        if z <= z1:
            t = (z - z0) / (z1 - z0)
            return (rx0 if idx == 1 else ry0) * (1.0 - t) + (rx1 if idx == 1 else ry1) * t
    return stations[-1][idx]


def _dress_form_torso_geometry() -> MeshGeometry:
    """Closed organic torso mesh: waist, bust, sloped shoulders, and short neck."""

    rings = [
        (0.10, 0.115, 0.065),
        (0.14, 0.135, 0.072),
        (0.23, 0.142, 0.080),
        (0.35, 0.105, 0.070),
        (0.49, 0.155, 0.094),
        (0.63, 0.195, 0.105),
        (0.73, 0.250, 0.090),
        (0.80, 0.155, 0.070),
        (0.88, 0.066, 0.052),
    ]
    segs = 72
    geom = MeshGeometry()
    ring_indices: list[list[int]] = []

    for z, rx, ry in rings:
        indices = []
        for i in range(segs):
            a = 2.0 * math.pi * i / segs
            # Slightly superelliptic fabric form: broad front/back and softened sides.
            ca = math.cos(a)
            sa = math.sin(a)
            sx = math.copysign(abs(ca) ** 0.86, ca)
            sy = math.copysign(abs(sa) ** 0.94, sa)
            front_fullness = 1.0 + (0.10 if sy < 0.0 and 0.42 < z < 0.70 else 0.0)
            indices.append(geom.add_vertex(rx * sx, ry * sy * front_fullness, z))
        ring_indices.append(indices)

    for lower, upper in zip(ring_indices, ring_indices[1:]):
        for i in range(segs):
            j = (i + 1) % segs
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])

    bottom_center = geom.add_vertex(0.0, 0.0, rings[0][0] - 0.015)
    for i in range(segs):
        geom.add_face(bottom_center, ring_indices[0][(i + 1) % segs], ring_indices[0][i])

    top_center = geom.add_vertex(0.0, 0.0, rings[-1][0] + 0.005)
    for i in range(segs):
        geom.add_face(top_center, ring_indices[-1][i], ring_indices[-1][(i + 1) % segs])

    return geom


def _front_vertical_seam(x_offset: float = 0.0) -> MeshGeometry:
    points = []
    for z in [0.13, 0.23, 0.35, 0.49, 0.63, 0.77, 0.86]:
        rx = _radius_at(z, "x")
        ry = _radius_at(z, "y")
        x = x_offset * (0.45 + 0.55 * rx / 0.20)
        y = -ry * 1.015
        points.append((x, y, z))
    return tube_from_spline_points(points, radius=0.0032, samples_per_segment=10, radial_segments=10)


def _ellipse_loop(z: float, scale: float = 1.015) -> MeshGeometry:
    rx = _radius_at(z, "x") * scale
    ry = _radius_at(z, "y") * scale
    points = []
    for i in range(56):
        a = 2.0 * math.pi * i / 56
        points.append((rx * math.cos(a), ry * math.sin(a), z))
    return tube_from_spline_points(
        points,
        radius=0.0028,
        samples_per_segment=3,
        radial_segments=8,
        closed_spline=True,
    )


def _star_base_leg(angle: float) -> MeshGeometry:
    r0 = 0.035
    r1 = 0.335
    points = [
        (r0 * math.cos(angle), r0 * math.sin(angle), 0.075),
        (0.18 * math.cos(angle), 0.18 * math.sin(angle), 0.055),
        (r1 * math.cos(angle), r1 * math.sin(angle), 0.045),
    ]
    return tube_from_spline_points(points, radius=0.012, samples_per_segment=12, radial_segments=14)


def _wire_gauge_loop() -> MeshGeometry:
    # Side loop visible on many vintage dress-form stands.
    points = [
        (0.005, 0.000, 0.61),
        (0.120, 0.010, 0.63),
        (0.245, 0.012, 0.72),
        (0.250, 0.010, 0.75),
        (0.125, 0.006, 0.66),
        (0.005, 0.000, 0.64),
    ]
    return tube_from_spline_points(points, radius=0.004, samples_per_segment=10, radial_segments=10)


def _wood_arm(side: float) -> MeshGeometry:
    shoulder = (side * 0.238, -0.004, 0.675)
    elbow = (side * 0.252, -0.092, 0.485)
    wrist = (side * 0.066, -0.145, 0.335)
    return tube_from_spline_points(
        [shoulder, (side * 0.270, -0.050, 0.570), elbow, (side * 0.190, -0.130, 0.400), wrist],
        radius=0.021,
        samples_per_segment=10,
        radial_segments=16,
    )


def _wood_hand(side: float) -> MeshGeometry:
    base = (side * 0.060, -0.150, 0.328)
    points = [
        base,
        (side * 0.036, -0.165, 0.312),
        (side * 0.012, -0.165, 0.310),
    ]
    palm = tube_from_spline_points(points, radius=0.017, samples_per_segment=8, radial_segments=12)
    for k, dz in enumerate([-0.014, -0.006, 0.002, 0.010]):
        finger = tube_from_spline_points(
            [
                (side * (0.030 + 0.004 * k), -0.170, 0.310 + dz),
                (side * (0.010 + 0.002 * k), -0.194, 0.300 + dz),
                (side * (-0.010 + 0.001 * k), -0.206, 0.296 + dz),
            ],
            radius=0.0038,
            samples_per_segment=5,
            radial_segments=8,
        )
        palm.merge(finger)
    return palm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tailor_dummy_on_adjustable_stand",
        meta={"category_note": "Reference image and folder both read as a tailor's dummy/dress form."},
    )

    lower = model.part("lower_stand")

    lower.visual(Cylinder(0.043, 0.035), origin=Origin(xyz=(0.0, 0.0, 0.065)), material=DARK_METAL, name="central_hub")
    lower.visual(Sphere(0.046), origin=Origin(xyz=(0.0, 0.0, 0.082)), material=DARK_METAL, name="hub_dome")
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0 + math.radians(18.0)
        lower.visual(mesh_from_geometry(_star_base_leg(angle), f"spoke_{i}"), material=DARK_METAL, name=f"spoke_{i}")
        end_x = 0.345 * math.cos(angle)
        end_y = 0.345 * math.sin(angle)
        tangent = angle + math.pi / 2.0
        lower.visual(
            Box((0.022, 0.016, 0.042)),
            origin=Origin(xyz=(end_x, end_y, 0.042), rpy=(0.0, 0.0, angle)),
            material=DARK_METAL,
            name=f"caster_fork_{i}",
        )
        lower.visual(
            Cylinder(0.026, 0.018),
            origin=Origin(xyz=(end_x, end_y, 0.021), rpy=_cylinder_rpy_for_xy(tangent)),
            material=TRAY_MAT,
            name=f"caster_wheel_{i}",
        )

    lower.visual(Cylinder(0.014, 0.670), origin=Origin(xyz=(0.0, 0.0, 0.3925)), material=DARK_METAL, name="lower_pole")
    lower.visual(Cylinder(0.026, 0.320), origin=Origin(xyz=(0.0, 0.0, 0.88)), material=DARK_METAL, name="outer_sleeve")
    lower.visual(Cylinder(0.035, 0.025), origin=Origin(xyz=(0.0, 0.0, 1.026)), material=DARK_METAL, name="sleeve_collar")
    lower.visual(mesh_from_geometry(_wire_gauge_loop(), "wire_gauge_loop"), material=DARK_METAL, name="wire_gauge_loop")
    lower.visual(
        Box((0.205, 0.165, 0.012)),
        origin=Origin(xyz=(-0.210, -0.030, 0.560), rpy=(0.0, 0.0, math.radians(-8.0))),
        material=TRAY_MAT,
        name="side_tray",
    )
    lower.visual(
        Cylinder(0.008, 0.195),
        origin=Origin(xyz=(-0.098, -0.012, 0.556), rpy=_cylinder_rpy_for_xy(math.radians(-8.0))),
        material=DARK_METAL,
        name="tray_arm",
    )
    lower.visual(
        Box((0.240, 0.052, 0.030)),
        origin=Origin(xyz=(-0.105, -0.005, 0.560), rpy=(0.0, 0.0, math.radians(-8.0))),
        material=DARK_METAL,
        name="tray_bracket",
    )

    upper = model.part("upper_stage")
    upper.visual(Cylinder(0.011, 0.730), origin=Origin(xyz=(0.0, 0.0, 0.155)), material=DARK_METAL, name="inner_pole")
    upper.visual(Cylinder(0.040, 0.040), origin=Origin(xyz=(0.0, 0.0, 0.150)), material=DARK_METAL, name="torso_mount_collar")
    upper.visual(mesh_from_geometry(_dress_form_torso_geometry(), "torso_shell"), material=FABRIC, name="torso_shell")
    upper.visual(mesh_from_geometry(_front_vertical_seam(0.0), "center_front_seam"), material=SEAM, name="center_front_seam")
    upper.visual(mesh_from_geometry(_front_vertical_seam(0.065), "front_seam_0"), material=SEAM, name="front_seam_0")
    upper.visual(mesh_from_geometry(_front_vertical_seam(-0.065), "front_seam_1"), material=SEAM, name="front_seam_1")
    upper.visual(mesh_from_geometry(_ellipse_loop(0.350), "waist_seam"), material=SEAM, name="waist_seam")
    upper.visual(mesh_from_geometry(_ellipse_loop(0.725), "shoulder_seam"), material=SEAM, name="shoulder_seam")
    upper.visual(Cylinder(0.073, 0.030), origin=Origin(xyz=(0.0, 0.0, 0.908)), material=DARK_METAL, name="neck_cap")
    upper.visual(Cylinder(0.050, 0.090), origin=Origin(xyz=(0.0, 0.0, 0.855)), material=FABRIC, name="cloth_neck")
    for side, name_suffix in [(-1.0, "0"), (1.0, "1")]:
        upper.visual(Sphere(0.032), origin=Origin(xyz=(side * 0.244, -0.006, 0.678)), material=WORN_WOOD, name=f"shoulder_ball_{name_suffix}")
        upper.visual(mesh_from_geometry(_wood_arm(side), f"wood_arm_{name_suffix}"), material=WORN_WOOD, name=f"wood_arm_{name_suffix}")
        upper.visual(Sphere(0.026), origin=Origin(xyz=(side * 0.252, -0.092, 0.485)), material=WORN_WOOD, name=f"elbow_ball_{name_suffix}")
        upper.visual(Sphere(0.022), origin=Origin(xyz=(side * 0.066, -0.145, 0.335)), material=WORN_WOOD, name=f"wrist_ball_{name_suffix}")
        upper.visual(mesh_from_geometry(_wood_hand(side), f"wood_hand_{name_suffix}"), material=WORN_WOOD, name=f"wood_hand_{name_suffix}")

    clamp = model.part("clamp_knob")
    clamp.visual(
        Cylinder(0.006, 0.058),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK_METAL,
        name="knob_shaft",
    )
    clamp.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="lobed",
                grip=KnobGrip(style="ribbed", count=12, depth=0.0015),
                center=True,
            ),
            "clamp_knob_cap",
        ),
        origin=Origin(xyz=(0.062, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=TRAY_MAT,
        name="knob_cap",
    )

    finial = model.part("top_finial")
    finial.visual(Cylinder(0.010, 0.070), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=WORN_WOOD, name="finial_stem")
    finial.visual(Sphere(0.022), origin=Origin(xyz=(0.0, 0.0, 0.083)), material=WORN_WOOD, name="finial_knob")
    finial.visual(
        Box((0.018, 0.042, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.115), rpy=(0.0, 0.0, math.radians(12.0))),
        material=WORN_WOOD,
        name="finial_flat",
    )

    model.articulation(
        "stand_height",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.220),
    )
    model.articulation(
        "sleeve_knob",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=clamp,
        origin=Origin(xyz=(0.026, 0.0, 0.875)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "neck_finial",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=finial,
        origin=Origin(xyz=(0.0, 0.0, 0.923)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_stand")
    upper = object_model.get_part("upper_stage")
    clamp = object_model.get_part("clamp_knob")
    height = object_model.get_articulation("stand_height")
    knob = object_model.get_articulation("sleeve_knob")

    ctx.allow_overlap(
        lower,
        upper,
        elem_a="outer_sleeve",
        elem_b="inner_pole",
        reason="The narrow upper pole is intentionally represented as a retained telescoping member inside the stand sleeve proxy.",
    )
    ctx.allow_overlap(
        lower,
        upper,
        elem_a="sleeve_collar",
        elem_b="inner_pole",
        reason="The top collar is a solid proxy for a bored clamp ring surrounding the sliding height pole.",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="sliding pole is centered in sleeve",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="inner_pole",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="collapsed height retains pole insertion",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="sleeve_collar",
        margin=0.002,
        name="sliding pole passes through top collar",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="inner_pole",
        elem_b="sleeve_collar",
        min_overlap=0.020,
        name="top collar captures pole locally",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        positive_elem="torso_shell",
        negative_elem="outer_sleeve",
        min_gap=0.002,
        name="fabric torso sits above sleeve hardware",
    )
    ctx.expect_gap(
        clamp,
        lower,
        axis="x",
        positive_elem="knob_shaft",
        negative_elem="outer_sleeve",
        min_gap=-0.002,
        max_gap=0.003,
        name="clamp knob shaft seats at sleeve side",
    )

    rest = ctx.part_world_position(upper)
    with ctx.pose({height: 0.220, knob: math.pi / 2.0}):
        ctx.expect_within(
            upper,
            lower,
            axes="xy",
            inner_elem="inner_pole",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended pole remains centered",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="inner_pole",
            elem_b="outer_sleeve",
            min_overlap=0.065,
            name="extended height retains pole insertion",
        )
        extended = ctx.part_world_position(upper)

    ctx.check(
        "prismatic height adjustment raises the dress form",
        rest is not None and extended is not None and extended[2] > rest[2] + 0.20,
        details=f"rest={rest}, extended={extended}",
    )
    ctx.check(
        "model includes visible torso, stand, casters, seams, knob, and finial",
        all(object_model.get_part(name) is not None for name in ("lower_stand", "upper_stage", "clamp_knob", "top_finial")),
    )

    return ctx.report()


object_model = build_object_model()
