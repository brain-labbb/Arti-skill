from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


_TORSO_SECTIONS = (
    # z, width, front depth (-Y), back depth (+Y), superellipse power
    (0.000, 0.340, 0.100, 0.088, 2.5),
    (0.110, 0.365, 0.112, 0.096, 2.6),
    (0.250, 0.292, 0.086, 0.083, 2.8),
    (0.390, 0.392, 0.138, 0.104, 2.4),
    (0.530, 0.440, 0.118, 0.102, 2.6),
    (0.610, 0.382, 0.096, 0.088, 2.8),
    (0.665, 0.210, 0.074, 0.068, 2.5),
    (0.720, 0.140, 0.058, 0.055, 2.4),
)


def _sgn(value: float) -> float:
    return -1.0 if value < 0.0 else 1.0


def _section_at(z: float) -> tuple[float, float, float, float]:
    sections = _TORSO_SECTIONS
    if z <= sections[0][0]:
        _, w, front, back, p = sections[0]
        return w, front, back, p
    if z >= sections[-1][0]:
        _, w, front, back, p = sections[-1]
        return w, front, back, p

    for lower, upper in zip(sections[:-1], sections[1:]):
        zl, wl, fl, bl, pl = lower
        zu, wu, fu, bu, pu = upper
        if zl <= z <= zu:
            t = (z - zl) / (zu - zl)
            return (
                wl + (wu - wl) * t,
                fl + (fu - fl) * t,
                bl + (bu - bl) * t,
                pl + (pu - pl) * t,
            )
    _, w, front, back, p = sections[-1]
    return w, front, back, p


def _torso_surface(theta: float, z: float) -> tuple[float, float, tuple[float, float, float]]:
    width, front_depth, back_depth, power = _section_at(z)
    c = math.cos(theta)
    s = math.sin(theta)
    exp = 2.0 / power
    x = 0.5 * width * _sgn(c) * (abs(c) ** exp)
    depth = front_depth if s < 0.0 else back_depth
    y = depth * _sgn(s) * (abs(s) ** exp)

    # A practical outward normal for strips and mounted hardware.
    nx = x / max(0.5 * width, 1e-6)
    ny = y / max(depth, 1e-6)
    length = math.hypot(nx, ny)
    if length < 1e-6:
        normal = (math.cos(theta), math.sin(theta), 0.0)
    else:
        normal = (nx / length, ny / length, 0.0)
    return x, y, normal


def _make_torso_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    segments = 72
    loops: list[list[int]] = []

    for z, width, front_depth, back_depth, power in _TORSO_SECTIONS:
        loop: list[int] = []
        exp = 2.0 / power
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            c = math.cos(theta)
            s = math.sin(theta)
            x = 0.5 * width * _sgn(c) * (abs(c) ** exp)
            depth = front_depth if s < 0.0 else back_depth
            y = depth * _sgn(s) * (abs(s) ** exp)
            loop.append(geom.add_vertex(x, y, z))
        loops.append(loop)

    for lower, upper in zip(loops[:-1], loops[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])

    bottom_center = geom.add_vertex(0.0, 0.0, _TORSO_SECTIONS[0][0])
    top_center = geom.add_vertex(0.0, 0.0, _TORSO_SECTIONS[-1][0])
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(bottom_center, loops[0][j], loops[0][i])
        geom.add_face(top_center, loops[-1][i], loops[-1][j])
    return geom


def _make_vertical_seam(theta: float, z0: float, z1: float, *, width: float = 0.008) -> MeshGeometry:
    geom = MeshGeometry()
    samples = 24
    rings: list[list[int]] = []
    tangent = (-math.sin(theta), math.cos(theta), 0.0)
    for k in range(samples + 1):
        z = z0 + (z1 - z0) * k / samples
        x, y, normal = _torso_surface(theta, z)
        outer = (x + normal[0] * 0.004, y + normal[1] * 0.004, z)
        inner = (x - normal[0] * 0.002, y - normal[1] * 0.002, z)
        ring = [
            geom.add_vertex(outer[0] - tangent[0] * width / 2, outer[1] - tangent[1] * width / 2, z),
            geom.add_vertex(outer[0] + tangent[0] * width / 2, outer[1] + tangent[1] * width / 2, z),
            geom.add_vertex(inner[0] + tangent[0] * width / 2, inner[1] + tangent[1] * width / 2, z),
            geom.add_vertex(inner[0] - tangent[0] * width / 2, inner[1] - tangent[1] * width / 2, z),
        ]
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(4):
            j = (i + 1) % 4
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])
    # End caps.
    for ring in (rings[0], rings[-1]):
        geom.add_face(ring[0], ring[1], ring[2])
        geom.add_face(ring[0], ring[2], ring[3])
    return geom


def _make_waist_band(z: float, *, height: float = 0.006) -> MeshGeometry:
    geom = MeshGeometry()
    segments = 72
    rings: list[list[int]] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        x, y, normal = _torso_surface(theta, z)
        outer = (x + normal[0] * 0.0035, y + normal[1] * 0.0035)
        inner = (x - normal[0] * 0.002, y - normal[1] * 0.002)
        rings.append(
            [
                geom.add_vertex(outer[0], outer[1], z - height / 2),
                geom.add_vertex(outer[0], outer[1], z + height / 2),
                geom.add_vertex(inner[0], inner[1], z + height / 2),
                geom.add_vertex(inner[0], inner[1], z - height / 2),
            ]
        )

    for i in range(segments):
        a = rings[i]
        b = rings[(i + 1) % segments]
        for j in range(4):
            n = (j + 1) % 4
            geom.add_face(a[j], b[j], b[n])
            geom.add_face(a[j], b[n], a[n])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_tailor_dummy")

    fabric = model.material("warm_grey_fabric", rgba=(0.30, 0.27, 0.27, 1.0))
    seam = model.material("black_recessed_seams", rgba=(0.018, 0.016, 0.017, 1.0))
    metal = model.material("brushed_chrome", rgba=(0.78, 0.76, 0.70, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.075, 0.07, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.024, 0.022, 1.0))
    brass = model.material("aged_brass", rgba=(0.70, 0.55, 0.32, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.045, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_metal,
        name="floor_hub",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=metal,
        name="outer_sleeve",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.672)),
        material=dark_metal,
        name="height_collar",
    )
    for idx, angle in enumerate((0.0, math.pi / 2, math.pi, 3 * math.pi / 2)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Cylinder(radius=0.0075, length=0.460),
            origin=Origin(
                xyz=(0.230 * c, 0.230 * s, 0.035),
                rpy=(0.0, math.pi / 2, angle),
            ),
            material=metal,
            name=f"tripod_leg_{idx}",
        )
        base.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(0.463 * c, 0.463 * s, 0.035)),
            material=rubber,
            name=f"foot_cap_{idx}",
        )

    height_pole = model.part("height_pole")
    height_pole.visual(
        Cylinder(radius=0.012, length=0.570),
        # The lower length is intentionally retained inside the sleeve.
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=metal,
        name="inner_pole",
    )
    height_pole.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=dark_metal,
        name="support_plate",
    )

    torso = model.part("torso")
    torso.visual(
        mesh_from_geometry(_make_torso_mesh(), "fabric_torso_shell"),
        material=fabric,
        name="torso_shell",
    )
    torso.visual(
        mesh_from_geometry(_make_vertical_seam(-math.pi / 2, 0.055, 0.705, width=0.009), "front_seam_mesh"),
        material=seam,
        name="front_seam",
    )
    torso.visual(
        mesh_from_geometry(_make_vertical_seam(-2.55, 0.070, 0.600, width=0.007), "side_seam_mesh"),
        material=seam,
        name="side_seam",
    )
    torso.visual(
        mesh_from_geometry(_make_waist_band(0.252), "waist_band_mesh"),
        material=seam,
        name="waist_seam",
    )
    torso.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.728)),
        material=dark_metal,
        name="neck_cap",
    )
    torso.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.707)),
        material=dark_metal,
        name="neck_plug",
    )

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="clamp_screw",
    )
    collar_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.020,
                body_style="lobed",
                base_diameter=0.030,
                top_diameter=0.038,
                crown_radius=0.0015,
                grip=KnobGrip(style="ribbed", count=8, depth=0.0012),
            ),
            "collar_lobed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark_metal,
        name="clamp_knob",
    )

    neck_knob = model.part("neck_knob")
    neck_knob.visual(
        Cylinder(radius=0.008, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_metal,
        name="neck_stem",
    )
    neck_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.032,
                0.018,
                body_style="domed",
                crown_radius=0.004,
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            ),
            "top_domed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_metal,
        name="top_knob",
    )

    # 6 sizing thumbwheels: 3 z-bands (hip, waist, bust) × 2 (right-front + left-front)
    _front_dial_seats = (
        # (z_height, theta_on_torso)
        (0.170, -math.pi / 2 + 0.55),   # hip right-front
        (0.170, -math.pi / 2 - 0.55),   # hip left-front
        (0.350, -math.pi / 2 + 0.55),   # waist right-front
        (0.350, -math.pi / 2 - 0.55),   # waist left-front
        (0.520, -math.pi / 2 + 0.55),   # bust right-front
        (0.520, -math.pi / 2 - 0.55),   # bust left-front
    )
    for idx, (z, theta) in enumerate(_front_dial_seats):
        dial = model.part(f"front_dial_{idx}")
        dial.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=brass,
            name="dial_stem",
        )
        dial.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=brass,
            name="dial_face",
        )
        dial.visual(
            Box((0.020, 0.0024, 0.0020)),
            origin=Origin(xyz=(0.0, 0.0, 0.0208)),
            material=seam,
            name="screw_slot",
        )

    model.articulation(
        "base_to_height_pole",
        ArticulationType.PRISMATIC,
        parent=base,
        child=height_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "height_pole_to_torso",
        ArticulationType.FIXED,
        parent=height_pole,
        child=torso,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
    )
    model.articulation(
        "base_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar_knob,
        origin=Origin(xyz=(0.030, 0.0, 0.672), rpy=(0.0, math.pi / 2, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "torso_to_neck_knob",
        ArticulationType.CONTINUOUS,
        parent=torso,
        child=neck_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.736)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    for idx, (z, theta) in enumerate(_front_dial_seats):
        x, y, normal = _torso_surface(theta, z)
        yaw = math.atan2(normal[0], -normal[1])
        model.articulation(
            f"torso_to_front_dial_{idx}",
            ArticulationType.CONTINUOUS,
            parent=torso,
            child=f"front_dial_{idx}",
            origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    height_pole = object_model.get_part("height_pole")
    torso = object_model.get_part("torso")
    collar_knob = object_model.get_part("collar_knob")
    neck_knob = object_model.get_part("neck_knob")
    lift = object_model.get_articulation("base_to_height_pole")

    ctx.allow_overlap(
        base,
        height_pole,
        elem_a="outer_sleeve",
        elem_b="inner_pole",
        reason="The height-adjustment inner pole is intentionally retained inside the outer sleeve.",
    )
    ctx.allow_overlap(
        base,
        height_pole,
        elem_a="height_collar",
        elem_b="inner_pole",
        reason="The collar surrounds the sliding pole as the real clamp hardware does.",
    )
    for idx in range(6):
        ctx.allow_overlap(
            object_model.get_part(f"front_dial_{idx}"),
            torso,
            elem_a="dial_stem",
            elem_b="torso_shell",
            reason="The tiny adjustment screw stem intentionally enters the dress-form shell.",
        )
        ctx.expect_contact(
            object_model.get_part(f"front_dial_{idx}"),
            torso,
            elem_a="dial_stem",
            elem_b="torso_shell",
            contact_tol=0.006,
            name=f"front dial {idx} stem reaches torso shell",
        )

    ctx.expect_within(
        height_pole,
        base,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner pole is centered in sleeve",
    )
    ctx.expect_overlap(
        height_pole,
        base,
        axes="z",
        elem_a="inner_pole",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="collapsed pole remains inserted",
    )
    ctx.expect_gap(
        torso,
        height_pole,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem="torso_shell",
        negative_elem="support_plate",
        name="torso shell seats on support plate",
    )
    ctx.expect_contact(
        collar_knob,
        base,
        elem_a="clamp_screw",
        elem_b="height_collar",
        contact_tol=0.003,
        name="clamp screw reaches collar",
    )
    ctx.expect_contact(
        neck_knob,
        torso,
        elem_a="neck_stem",
        elem_b="neck_cap",
        contact_tol=0.003,
        name="top knob sits on neck cap",
    )

    rest_pos = ctx.part_world_position(torso)
    with ctx.pose({lift: 0.250}):
        ctx.expect_within(
            height_pole,
            base,
            axes="xy",
            inner_elem="inner_pole",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended pole stays centered",
        )
        ctx.expect_overlap(
            height_pole,
            base,
            axes="z",
            elem_a="inner_pole",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="extended pole keeps retained insertion",
        )
        raised_pos = ctx.part_world_position(torso)
    ctx.check(
        "prismatic height adjustment raises torso",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.22,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "6 sizing thumbwheels with continuous joints",
        all(
            object_model.get_part(f"front_dial_{i}") is not None
            and object_model.get_articulation(f"torso_to_front_dial_{i}") is not None
            for i in range(6)
        ),
        details="Expected front_dial_0..5 parts each with a torso_to_front_dial_{i} CONTINUOUS joint.",
    )
    ctx.check(
        "visible dress form controls are articulated",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "base_to_collar_knob",
                "torso_to_neck_knob",
                "torso_to_front_dial_0",
                "torso_to_front_dial_1",
                "torso_to_front_dial_2",
                "torso_to_front_dial_3",
                "torso_to_front_dial_4",
                "torso_to_front_dial_5",
            )
        ),
        details="Expected clamp, neck, and all 6 front adjustment dials to have rotary joints.",
    )

    return ctx.report()


object_model = build_object_model()
