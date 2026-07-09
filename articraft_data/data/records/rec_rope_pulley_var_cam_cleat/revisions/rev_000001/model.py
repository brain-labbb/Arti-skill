from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, center=(0.0, 0.0), segments: int = 48):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _offset_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _pulley_cheek_mesh(name: str):
    """Oval stainless side cheek with a side viewing window for the sheave."""
    outer = superellipse_profile(0.058, 0.116, exponent=2.35, segments=72)
    sheave_window = _offset_profile(
        rounded_rect_profile(0.021, 0.066, radius=0.008, corner_segments=8),
        -0.018,
        -0.006,
    )
    top_relief = _offset_profile(
        rounded_rect_profile(0.026, 0.018, radius=0.006, corner_segments=8),
        0.0,
        0.037,
    )
    plate = ExtrudeWithHolesGeometry(
        outer,
        [sheave_window, top_relief],
        0.004,
        cap=True,
        center=True,
        closed=True,
    )
    # Mesh extrudes along local Z.  Rotate so the plate face is XZ and thickness
    # is along Y, matching the pulley axle.
    plate.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(plate, name)


def _cam_jaw_mesh(name: str):
    """Toothed cam jaw for one-way rope gripping.

    Eccentric lobe profile: small pivot boss (base_r=0.004) with a large
    toothed lobe extending toward local -X (rope-gripping face).  The small
    base radius ensures the cam body can fully clear the rope at the release
    angle, while the lobe reaches into the rope for self-locking grip at rest.

    The profile is extruded along local Z, then rotated so thickness is along
    Y to match the sheave axle direction.
    """
    base_r = 0.004
    lobe_extent = 0.009
    tooth_depth = 0.002
    n_teeth = 6
    grip_arc = math.pi * 0.45  # ±81° around the gripping face

    points: list[tuple[float, float]] = []
    n = 120
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        r = base_r
        # Gripping face centered at theta = π (local -X)
        angle_from_grip = abs(
            ((theta - math.pi) + math.pi) % (2.0 * math.pi) - math.pi
        )
        if angle_from_grip < grip_arc:
            blend = 1.0 - (angle_from_grip / grip_arc) ** 2
            lobe = lobe_extent * blend
            tooth_phase = angle_from_grip * n_teeth / grip_arc
            tooth = tooth_depth * abs(math.sin(tooth_phase * math.pi)) * blend
            r += lobe + tooth
        points.append((r * math.cos(theta), r * math.sin(theta)))

    pivot_hole = _circle_profile(0.0025, segments=24)
    cam = ExtrudeWithHolesGeometry(
        points, [pivot_hole], 0.010, cap=True, center=True, closed=True
    )
    cam.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(cam, name)


def _rope_mesh():
    # A short black rope run: two vertical legs with a smooth bend over the
    # sheave footprint, representing the rope threaded through the pulley.
    radius = 0.0248
    points = [(-radius, 0.0, -0.135), (-radius, 0.0, -0.060)]
    for i in range(17):
        theta = math.pi - math.pi * i / 16.0
        points.append((radius * math.cos(theta), 0.0, radius * math.sin(theta)))
    points.extend([(radius, 0.0, -0.060), (radius, 0.0, -0.135)])
    rope = tube_from_spline_points(
        points,
        radius=0.0047,
        samples_per_segment=6,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(rope, "threaded_rope")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hanging_rope_pulley",
        meta={
            "run_notes": (
                "Self-locking rope ratchet pulley variant: stainless housing "
                "with oval cheek plates, exposed grooved sheave on continuous "
                "axle, top swivel connector, threaded rope, plus a "
                "black-anodized spring-loaded cam cleat jaw for one-way "
                "tensioning. The cam teeth pinch the rope exiting the sheave "
                "groove; the sheave still turns freely under the rope."
            )
        },
    )

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = model.material("shadowed_groove", rgba=(0.10, 0.10, 0.10, 1.0))
    black_rope = model.material("black_braided_rope", rgba=(0.015, 0.014, 0.012, 1.0))
    polished = model.material("polished_edges", rgba=(0.92, 0.91, 0.86, 1.0))
    black_anodized = model.material("black_anodized_cam", rgba=(0.06, 0.06, 0.07, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.55, 0.56, 0.52, 1.0))

    housing = model.part("housing")
    cheek_mesh = _pulley_cheek_mesh("oval_cheek")
    for y, name in ((-0.014, "front_cheek"), (0.014, "rear_cheek")):
        housing.visual(
            cheek_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=stainless,
            name=name,
        )

    # Cross members tie the two oval side cheeks into a real yoke so the side
    # housing reads as one supported metal assembly.
    housing.visual(
        Cylinder(radius=0.0065, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="top_spacer",
    )
    housing.visual(
        Cylinder(radius=0.0042, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.057), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="bottom_pin",
    )
    housing.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=stainless,
        name="swivel_socket",
    )
    housing.visual(
        Cylinder(radius=0.0032, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle_pin",
    )

    # Axle screw heads on the outside of the cheeks; the actual sheave axle is
    # represented by the revolute joint so the wheel can spin freely.
    for y, name in ((-0.018, "front_axle_fastener"), (0.018, "rear_axle_fastener")):
        housing.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=name,
        )
        housing.visual(
            Cylinder(radius=0.0048, length=0.0048),
            origin=Origin(xyz=(0.0, y * 1.03, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"{name}_boss",
        )

    # Raised "15" load marking on the front cheek, approximated as separate
    # proud metal strokes seated on the plate face.
    text_y = -0.0168
    housing.visual(
        Box((0.0028, 0.0018, 0.023)),
        origin=Origin(xyz=(0.007, text_y, 0.026)),
        material=polished,
        name="marking_1",
    )
    for i, (x, z, sx, sz) in enumerate(
        [
            (0.017, 0.037, 0.013, 0.0026),
            (0.011, 0.030, 0.0026, 0.014),
            (0.017, 0.023, 0.013, 0.0026),
            (0.023, 0.016, 0.0026, 0.014),
            (0.017, 0.009, 0.013, 0.0026),
        ]
    ):
        housing.visual(
            Box((sx, 0.0018, sz)),
            origin=Origin(xyz=(x, text_y, z)),
            material=polished,
            name=f"marking_5_{i}",
        )

    # --- Cam cleat pivot hardware on the housing ---
    # The cam jaw rotates on its own axle, parallel to the sheave axle.
    # Pivot is offset to the right side of the sheave and below the sheave
    # center, so the cam teeth swing into the exiting rope on one side.
    cam_pivot_xyz = (0.042, 0.0, -0.048)

    # Cam mounting bracket: a vertical plate connecting the lower cheek region
    # to the cam pivot area, providing real structural support for the cam.
    housing.visual(
        Box((0.020, 0.004, 0.030)),
        origin=Origin(xyz=(0.036, -0.014, -0.040)),
        material=stainless,
        name="cam_bracket_front",
    )
    housing.visual(
        Box((0.020, 0.004, 0.030)),
        origin=Origin(xyz=(0.036, 0.014, -0.040)),
        material=stainless,
        name="cam_bracket_rear",
    )

    # Cam pivot axle spans between the brackets, embedding into both.
    housing.visual(
        Cylinder(radius=0.0025, length=0.032),
        origin=Origin(xyz=cam_pivot_xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="cam_pivot_axle",
    )
    # Torsion spring anchor: a small boss on the front bracket representing
    # the spring return that pushes the jaw toward the rope.
    housing.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.046, -0.012, -0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spring_steel,
        name="cam_spring_anchor",
    )

    cam_jaw = model.part("cam_jaw")
    cam_jaw.visual(
        _cam_jaw_mesh("cam_body"),
        origin=Origin(),
        material=black_anodized,
        name="cam_body",
    )
    # Cam bushing: a small cylinder at the pivot bore so the jaw reads as
    # mounted on the housing axle.
    cam_jaw.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="cam_bushing",
    )

    upper_swivel = model.part("upper_swivel")
    upper_swivel.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(),
        material=stainless,
        name="swivel_collar",
    )
    upper_swivel.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="connector_barrel",
    )
    upper_swivel.visual(
        Cylinder(radius=0.0038, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="clip_base_pin",
    )

    # A stainless carabiner/clip connection above the swivel collar.  This whole
    # upper connector rotates together on the vertical swivel axis.
    clip_points = [
        (-0.020, 0.0, 0.011),
        (-0.024, 0.0, 0.035),
        (-0.017, 0.0, 0.061),
        (0.0, 0.0, 0.071),
        (0.017, 0.0, 0.061),
        (0.024, 0.0, 0.035),
        (0.020, 0.0, 0.011),
    ]
    clip = tube_from_spline_points(
        clip_points,
        radius=0.0038,
        samples_per_segment=10,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    upper_swivel.visual(
        mesh_from_geometry(clip, "top_clip_loop"),
        material=polished,
        name="top_clip_loop",
    )
    upper_swivel.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.014, 0.0, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="gate_hinge_knuckle",
    )

    sheave = model.part("sheave")
    # Three coaxial cylinders make a grooved pulley: bright flanges with a
    # darker smaller-diameter rope groove in between.
    sheave.visual(
        Cylinder(radius=0.0215, length=0.006),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="front_flange",
    )
    sheave.visual(
        Cylinder(radius=0.0168, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rope_groove",
    )
    sheave.visual(
        Cylinder(radius=0.0215, length=0.006),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="rear_flange",
    )
    sheave.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hub_bushing",
    )

    rope = model.part("rope")
    rope.visual(_rope_mesh(), material=black_rope, name="threaded_rope")

    model.articulation(
        "housing_to_sheave",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=sheave,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "housing_to_rope",
        ArticulationType.FIXED,
        parent=housing,
        child=rope,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_upper_swivel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=upper_swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    model.articulation(
        "housing_to_cam",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cam_jaw,
        # Cam pivot offset to the right and below the sheave center.
        # Axis parallels the sheave axle (Y).  Positive q rotates the toothed
        # face away from the exiting rope (release against spring pressure).
        origin=Origin(xyz=cam_pivot_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=-0.10, upper=0.90
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    sheave = object_model.get_part("sheave")
    rope = object_model.get_part("rope")
    upper_swivel = object_model.get_part("upper_swivel")
    cam_jaw = object_model.get_part("cam_jaw")
    sheave_joint = object_model.get_articulation("housing_to_sheave")
    upper_swivel_joint = object_model.get_articulation("housing_to_upper_swivel")
    cam_joint = object_model.get_articulation("housing_to_cam")

    ctx.allow_overlap(
        housing,
        sheave,
        elem_a="axle_pin",
        elem_b="hub_bushing",
        reason=(
            "The stationary axle pin is intentionally captured inside the "
            "rotating sheave bushing so the pulley has a realistic supported "
            "spin axis."
        ),
    )
    ctx.allow_overlap(
        housing,
        cam_jaw,
        elem_a="cam_pivot_axle",
        elem_b="cam_bushing",
        reason=(
            "The cam pivot axle is intentionally captured inside the cam "
            "bushing so the jaw rotates on a real supported axis."
        ),
    )
    ctx.allow_overlap(
        cam_jaw,
        rope,
        elem_a="cam_body",
        elem_b="threaded_rope",
        reason=(
            "The cam teeth are designed to pinch and grip the rope for "
            "one-way self-locking tensioning; small local contact overlap "
            "represents the seated gripping state."
        ),
    )
    ctx.allow_overlap(
        housing,
        upper_swivel,
        elem_a="swivel_socket",
        elem_b="swivel_collar",
        reason=(
            "The swivel collar seats into the housing swivel socket as a "
            "realistic bearing interface for the top connector rotation."
        ),
    )

    ctx.check(
        "image classification note",
        "rope pulley" in object_model.name.replace("_", " "),
        details="Source image and category both indicate a rope pulley.",
    )
    ctx.check(
        "sheave has axle rotation",
        tuple(round(v, 3) for v in sheave_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={sheave_joint.axis}",
    )
    ctx.check(
        "upper connector has vertical continuous swivel",
        upper_swivel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in upper_swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={upper_swivel_joint.articulation_type}, axis={upper_swivel_joint.axis}",
    )
    # --- Cam cleat assertions ---
    ctx.check(
        "cam_jaw has revolute joint parallel to sheave axle",
        cam_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in cam_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={cam_joint.articulation_type}, axis={cam_joint.axis}",
    )
    ctx.check(
        "cam_body has toothed gripping geometry",
        any(v.name == "cam_body" for v in cam_jaw.visuals),
        details="cam_jaw must carry a toothed cam_body visual for one-way rope ratchet.",
    )
    # At rest (q=0), cam teeth contact the rope.
    ctx.expect_contact(
        cam_jaw,
        rope,
        elem_a="cam_body",
        elem_b="threaded_rope",
        contact_tol=0.005,
        name="cam teeth reach the rope at spring rest",
    )
    # Cam pivot axle captured inside cam bushing.
    ctx.expect_overlap(
        housing,
        cam_jaw,
        axes="xyz",
        elem_a="cam_pivot_axle",
        elem_b="cam_bushing",
        min_overlap=0.004,
        name="cam pivot axle passes through cam bushing",
    )
    # At max release, the cam rotates away from the rope.
    with ctx.pose({cam_joint: 0.90}):
        ctx.expect_gap(
            cam_jaw,
            rope,
            axis="x",
            max_penetration=0.001,
            positive_elem="cam_body",
            negative_elem="threaded_rope",
            name="cam releases rope at max open angle",
        )

    ctx.expect_within(
        sheave,
        housing,
        axes="xy",
        margin=0.002,
        name="sheave centered between cheek plates",
    )
    ctx.expect_overlap(
        housing,
        sheave,
        axes="xyz",
        elem_a="axle_pin",
        elem_b="hub_bushing",
        min_overlap=0.006,
        name="axle pin passes through sheave bushing",
    )
    ctx.expect_overlap(
        rope,
        sheave,
        axes="xz",
        min_overlap=0.038,
        name="rope projects through sheave path",
    )
    ctx.expect_overlap(
        upper_swivel,
        housing,
        axes="z",
        min_overlap=0.006,
        name="upper swivel collar seats in housing socket",
    )

    rope_aabb = ctx.part_world_aabb(rope)
    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "rope hangs below pulley body",
        rope_aabb is not None
        and housing_aabb is not None
        and rope_aabb[0][2] < housing_aabb[0][2] - 0.050,
        details=f"rope_aabb={rope_aabb}, housing_aabb={housing_aabb}",
    )

    centered_upper = ctx.part_world_aabb(upper_swivel)
    with ctx.pose({upper_swivel_joint: math.pi / 2.0}):
        rotated_upper = ctx.part_world_aabb(upper_swivel)
    ctx.check(
        "upper connector swivels as one assembly",
        centered_upper is not None
        and rotated_upper is not None
        and abs(
            (rotated_upper[1][1] - rotated_upper[0][1])
            - (centered_upper[1][0] - centered_upper[0][0])
        )
        < 0.010
        and abs(rotated_upper[1][2] - centered_upper[1][2]) < 0.002,
        details=f"centered={centered_upper}, rotated={rotated_upper}",
    )

    with ctx.pose({sheave_joint: math.pi / 2.0}):
        ctx.expect_within(
            sheave,
            housing,
            axes="xy",
            margin=0.002,
            name="rotated sheave remains captured",
        )

    return ctx.report()


object_model = build_object_model()
