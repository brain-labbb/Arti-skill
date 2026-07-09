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
                "Modeled the source image as a small hanging stainless rope "
                "pulley with an oval cheek housing, top connector, exposed "
                "sheave, axle fasteners, and a short threaded rope segment. "
                "The category name appears consistent with the image."
            )
        },
    )

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_steel = model.material("shadowed_groove", rgba=(0.10, 0.10, 0.10, 1.0))
    black_rope = model.material("black_braided_rope", rgba=(0.015, 0.014, 0.012, 1.0))
    polished = model.material("polished_edges", rgba=(0.92, 0.91, 0.86, 1.0))

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

    # --- Upper swivel: snap-shackle / spring-gated clevis head ---
    # Replaces the carabiner clip loop with a forged D-bail body and a
    # hinged spring gate (separate shackle_gate part).  Slightly beefier
    # proportions and polished-stainless colorway for the sailing variant.
    upper_swivel = model.part("upper_swivel")
    upper_swivel.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(),
        material=polished,
        name="swivel_collar",
    )
    # Short vertical neck connecting the swivel collar to the shackle bail
    upper_swivel.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=polished,
        name="connector_barrel",
    )
    # Crosspin at the base of the bail for realistic forged-body attachment
    upper_swivel.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="shackle_base_pin",
    )

    # Snap-shackle bail: a D-shaped bow open on the +X side.
    # The gap between the hinge point (top) and catch point (bottom) is
    # closed by the spring gate part when the joint is at rest (q=0).
    bail_points = [
        (0.014, 0.0, 0.040),   # hinge end (top of gate opening)
        (0.017, 0.0, 0.052),   # upper right
        (0.012, 0.0, 0.062),   # right shoulder
        (0.0, 0.0, 0.070),     # top of bow
        (-0.016, 0.0, 0.062),  # left shoulder
        (-0.021, 0.0, 0.046),  # widest left
        (-0.016, 0.0, 0.030),  # lower left
        (0.0, 0.0, 0.024),     # bottom center
        (0.010, 0.0, 0.024),   # approach catch
        (0.014, 0.0, 0.022),   # catch end (bottom of gate opening)
    ]
    bail = tube_from_spline_points(
        bail_points,
        radius=0.005,
        samples_per_segment=10,
        radial_segments=20,
        cap_ends=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    upper_swivel.visual(
        mesh_from_geometry(bail, "shackle_body"),
        material=polished,
        name="shackle_body",
    )
    # Hinge pin crossing the bail at the top of the gate opening
    upper_swivel.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.014, 0.0, 0.040), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="shackle_hinge_pin",
    )

    # --- Spring gate (separate revolute child of upper_swivel) ---
    shackle_gate = model.part("shackle_gate")
    # Gate lever bar: extends downward from hinge to catch when closed
    _gate_len = 0.018
    shackle_gate.visual(
        Cylinder(radius=0.004, length=_gate_len),
        origin=Origin(xyz=(0.0, 0.0, -_gate_len / 2.0)),
        material=polished,
        name="gate_lever",
    )
    # Plunger tip at the free (catch) end of the gate
    shackle_gate.visual(
        Cylinder(radius=0.005, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -_gate_len - 0.001)),
        material=stainless,
        name="gate_plunger_tip",
    )
    # Small release tab on the outside of the gate for thumb operation
    shackle_gate.visual(
        Box((0.008, 0.005, 0.004)),
        origin=Origin(xyz=(0.003, 0.0, -_gate_len * 0.55)),
        material=stainless,
        name="gate_release_tab",
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
    # Spring-loaded gate: pivots around Y axis at the hinge point.
    # axis=(0,-1,0) so positive q swings the gate bottom outward (+X).
    model.articulation(
        "upper_swivel_to_gate",
        ArticulationType.REVOLUTE,
        parent=upper_swivel,
        child=shackle_gate,
        origin=Origin(xyz=(0.014, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=6.0, lower=0.0, upper=0.60,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    sheave = object_model.get_part("sheave")
    rope = object_model.get_part("rope")
    upper_swivel = object_model.get_part("upper_swivel")
    shackle_gate = object_model.get_part("shackle_gate")
    sheave_joint = object_model.get_articulation("housing_to_sheave")
    upper_swivel_joint = object_model.get_articulation("housing_to_upper_swivel")
    gate_joint = object_model.get_articulation("upper_swivel_to_gate")

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
        upper_swivel,
        elem_a="swivel_socket",
        elem_b="swivel_collar",
        reason=(
            "The swivel collar is intentionally nested inside the housing "
            "swivel socket to represent the rotating connection between the "
            "pulley body and the upper shackle head."
        ),
    )
    ctx.allow_overlap(
        upper_swivel,
        shackle_gate,
        elem_a="shackle_body",
        elem_b="gate_lever",
        reason=(
            "The spring gate lever is intentionally seated within the bail "
            "tube endpoints at the hinge point; at rest the gate closes the "
            "shackle gap with its pivot bore captured at the hinge."
        ),
    )
    ctx.allow_overlap(
        upper_swivel,
        shackle_gate,
        elem_a="shackle_body",
        elem_b="gate_plunger_tip",
        reason=(
            "The gate plunger tip is intentionally nested within the bail "
            "tube at the catch point, representing the spring-loaded detent "
            "that retains the closed gate."
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
    # --- Snap-shackle gate assertions (variant-specific) ---
    ctx.check(
        "snap shackle gate is revolute",
        gate_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in gate_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={gate_joint.articulation_type}, axis={gate_joint.axis}",
    )
    ctx.check(
        "snap shackle bail body exists on upper_swivel",
        upper_swivel.get_visual("shackle_body") is not None,
        details="The D-bail shackle_body mesh is the primary visible geometry "
                "of the snap-shackle variant.",
    )
    ctx.expect_overlap(
        upper_swivel,
        shackle_gate,
        axes="z",
        elem_a="shackle_body",
        elem_b="gate_lever",
        min_overlap=0.010,
        name="gate lever spans the bail opening vertically",
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

    # Gate opens outward: confirm the gate AABB extends in +X when opened
    gate_rest_aabb = ctx.part_world_aabb(shackle_gate)
    with ctx.pose({gate_joint: 0.55}):
        gate_open_aabb = ctx.part_world_aabb(shackle_gate)
    ctx.check(
        "snap shackle gate swings outward when opened",
        gate_rest_aabb is not None
        and gate_open_aabb is not None
        and gate_open_aabb[1][0] > gate_rest_aabb[1][0] + 0.002,
        details=f"rest_max_x={gate_rest_aabb[1][0]:.4f}, open_max_x={gate_open_aabb[1][0]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
