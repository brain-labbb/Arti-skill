from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float], *, fillet: float = 0.0):
    shape = cq.Workplane("XY").box(*size).translate(center)
    if fillet > 0.0:
        # Vertical-edge fillets give the machined aluminum blocks the rounded
        # die-cast look visible in the reference without weakening flat mounts.
        shape = shape.edges("|Z").fillet(fillet)
    return shape


def _metal_manifold_shape():
    """Single connected aluminum frame: top plate, standoffs, actuator blocks, and clevis cheeks."""
    shape = _cq_box((0.118, 0.032, 0.008), (0.0, 0.0, 0.140), fillet=0.003)
    shape = shape.union(_cq_box((0.030, 0.080, 0.008), (0.0, 0.0, 0.140), fillet=0.003))
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            shape = shape.union(_cq_box((0.030, 0.026, 0.008), (sx * 0.043, sy * 0.027, 0.140), fillet=0.004))

    # Central pneumatic/manifold body and side bridges.
    shape = shape.union(_cq_box((0.036, 0.034, 0.039), (0.0, 0.0, 0.114), fillet=0.003))
    shape = shape.union(_cq_box((0.080, 0.018, 0.018), (0.0, 0.0, 0.122), fillet=0.002))

    # Two opposing actuator bases below the plate, matching the visible side blocks.
    for sx in (-1.0, 1.0):
        shape = shape.union(_cq_box((0.048, 0.052, 0.040), (sx * 0.052, 0.0, 0.103), fillet=0.006))
        shape = shape.union(_cq_box((0.014, 0.014, 0.018), (sx * 0.040, 0.020, 0.130), fillet=0.002))
        shape = shape.union(_cq_box((0.014, 0.014, 0.018), (sx * 0.040, -0.020, 0.130), fillet=0.002))
        # Dovetail slide rail plate on actuator underside for carriage travel.
        shape = shape.union(_cq_box((0.050, 0.026, 0.006), (sx * 0.052, 0.0, 0.084), fillet=0.001))
        # Rail channel groove below the plate captures carriage dovetail tongue.
        shape = shape.union(_cq_box((0.034, 0.020, 0.005), (sx * 0.052, 0.0, 0.0795), fillet=0.001))

    return shape


def _finger_shape(inward_sign: float):
    """Soft silicone pneumatic finger with connected ribbed bellows."""
    shape = _cq_box((0.026, 0.030, 0.012), (0.0, 0.0, -0.006), fillet=0.003)
    shape = shape.union(_cq_box((0.016, 0.027, 0.074), (-inward_sign * 0.004, 0.0, -0.046), fillet=0.004))

    for i in range(5):
        z = -0.021 - i * 0.0125
        # Rounded external bellows fins protrude toward the gripping gap.
        shape = shape.union(_cq_box((0.031, 0.036, 0.009), (inward_sign * 0.006, 0.0, z), fillet=0.0045))

    shape = shape.union(_cq_box((0.020, 0.028, 0.014), (inward_sign * 0.001, 0.0, -0.086), fillet=0.005))
    return shape


def _carriage_shape():
    """Sliding carriage block with dovetail tongue, mount boss, and clevis cheeks."""
    # Main carriage body
    shape = _cq_box((0.044, 0.048, 0.012), (0.0, 0.0, 0.0), fillet=0.003)
    # Dovetail tongue on top engages the manifold rail groove.
    shape = shape.union(_cq_box((0.030, 0.018, 0.005), (0.0, 0.0, 0.0085), fillet=0.001))
    # Finger mount boss on bottom.
    shape = shape.union(_cq_box((0.026, 0.030, 0.004), (0.0, 0.0, -0.008), fillet=0.002))
    # Clevis cheeks capture the finger hinge pin.
    shape = shape.union(_cq_box((0.028, 0.005, 0.016), (0.0, 0.026, -0.004), fillet=0.001))
    shape = shape.union(_cq_box((0.028, 0.005, 0.016), (0.0, -0.026, -0.004), fillet=0.001))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_pneumatic_gripper")

    aluminum = model.material("satin_aluminum", rgba=(0.78, 0.80, 0.80, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.02, 0.02, 0.022, 1.0))
    silicone = model.material("matte_black_silicone", rgba=(0.01, 0.011, 0.012, 1.0))
    nylon = model.material("white_nylon_tube", rgba=(0.86, 0.88, 0.84, 1.0))
    steel = model.material("brushed_screw_heads", rgba=(0.62, 0.64, 0.65, 1.0))

    manifold = model.part("manifold")
    manifold.visual(
        mesh_from_cadquery(_metal_manifold_shape(), "machined_manifold"),
        material=aluminum,
        name="machined_manifold",
    )

    # Top plate holes and fasteners as visible circular details on the metal plate.
    for idx, (x, y) in enumerate([(-0.043, -0.027), (-0.043, 0.027), (0.043, -0.027), (0.043, 0.027)]):
        manifold.visual(
            Cylinder(radius=0.0042, length=0.003),
            origin=Origin(xyz=(x, y, 0.1455)),
            material=steel,
            name=f"screw_{idx}",
        )
        manifold.visual(
            Cylinder(radius=0.0020, length=0.0032),
            origin=Origin(xyz=(x, y, 0.1470)),
            material=dark_recess,
            name=f"hex_socket_{idx}",
        )

    for idx, (x, y) in enumerate([(-0.019, 0.0), (0.019, 0.0), (0.0, -0.017), (0.0, 0.017)]):
        manifold.visual(
            Cylinder(radius=0.0022, length=0.0015),
            origin=Origin(xyz=(x, y, 0.14475)),
            material=dark_recess,
            name=f"port_hole_{idx}",
        )

    for idx, x in enumerate((-0.036, 0.036)):
        manifold.visual(
            Cylinder(radius=0.0040, length=0.010),
            origin=Origin(xyz=(x, 0.009, 0.149)),
            material=nylon,
            name=f"barb_fitting_{idx}",
        )

    fixed_tube = tube_from_spline_points(
        [
            (-0.036, 0.009, 0.154),
            (-0.030, 0.030, 0.132),
            (0.0, 0.037, 0.110),
            (0.030, 0.030, 0.132),
            (0.036, 0.009, 0.154),
        ],
        radius=0.0024,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    manifold.visual(
        mesh_from_geometry(fixed_tube, "fixed_air_tube"),
        material=nylon,
        name="fixed_air_tube",
    )
    manifold.visual(
        Cylinder(radius=0.0040, length=0.009),
        origin=Origin(xyz=(0.0, -0.002, 0.0905)),
        material=nylon,
        name="swivel_socket",
    )

    # Sliding carriages for span adjustment along the manifold width axis.
    carriage_0 = model.part("finger_carriage_0")
    carriage_0.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_0_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="carriage_body",
    )

    carriage_1 = model.part("finger_carriage_1")
    carriage_1.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_1_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="carriage_body",
    )

    finger_0 = model.part("finger_0")
    finger_0.visual(
        mesh_from_cadquery(_finger_shape(1.0), "finger_0_bellows"),
        material=silicone,
        name="bellows",
    )

    finger_1 = model.part("finger_1")
    finger_1.visual(
        mesh_from_cadquery(_finger_shape(-1.0), "finger_1_bellows"),
        material=silicone,
        name="bellows",
    )

    hose_swivel = model.part("hose_swivel")
    swivel_tube = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, -0.002, -0.012),
            (0.0, -0.012, -0.021),
            (0.0, -0.026, -0.025),
        ],
        radius=0.0030,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    hose_swivel.visual(
        mesh_from_geometry(swivel_tube, "hose_swivel_elbow"),
        material=nylon,
        name="elbow_tube",
    )

    # Prismatic span-adjust slides: each carriage moves along X to widen/narrow grip.
    slide_limits = MotionLimits(effort=12.0, velocity=0.05, lower=0.0, upper=0.030)
    model.articulation(
        "manifold_to_carriage_0",
        ArticulationType.PRISMATIC,
        parent=manifold,
        child=carriage_0,
        origin=Origin(xyz=(-0.052, 0.0, 0.074)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=slide_limits,
    )
    model.articulation(
        "manifold_to_carriage_1",
        ArticulationType.PRISMATIC,
        parent=manifold,
        child=carriage_1,
        origin=Origin(xyz=(0.052, 0.0, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=slide_limits,
        mimic=Mimic(joint="manifold_to_carriage_0", multiplier=1.0, offset=0.0),
    )

    # Revolute finger bends re-parented onto carriages so bending stacks on slide.
    finger_limits = MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.55)
    model.articulation(
        "finger_0_bend",
        ArticulationType.REVOLUTE,
        parent=carriage_0,
        child=finger_0,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=finger_limits,
    )
    model.articulation(
        "finger_1_bend",
        ArticulationType.REVOLUTE,
        parent=carriage_1,
        child=finger_1,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=finger_limits,
        mimic=Mimic(joint="finger_0_bend", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "hose_swivel",
        ArticulationType.REVOLUTE,
        parent=manifold,
        child=hose_swivel,
        origin=Origin(xyz=(0.0, -0.002, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    manifold = object_model.get_part("manifold")
    carriage_0 = object_model.get_part("finger_carriage_0")
    carriage_1 = object_model.get_part("finger_carriage_1")
    finger_0 = object_model.get_part("finger_0")
    finger_1 = object_model.get_part("finger_1")
    hose = object_model.get_part("hose_swivel")
    slide_0 = object_model.get_articulation("manifold_to_carriage_0")
    slide_1 = object_model.get_articulation("manifold_to_carriage_1")
    bend = object_model.get_articulation("finger_0_bend")
    swivel = object_model.get_articulation("hose_swivel")

    ctx.check(
        "soft gripper has opposing bellows, carriages, and swivel hose",
        all(p is not None for p in (manifold, carriage_0, carriage_1, finger_0, finger_1, hose)),
        details="Expected manifold, two carriages, two bellows fingers, and a hose swivel.",
    )

    # Carriage dovetail tongue engages manifold rail groove (intentional nesting).
    ctx.allow_overlap(
        manifold,
        carriage_0,
        reason="Carriage dovetail tongue engages the manifold rail groove for prismatic slide.",
    )
    ctx.allow_overlap(
        manifold,
        carriage_1,
        reason="Carriage dovetail tongue engages the manifold rail groove for prismatic slide.",
    )
    # Finger neck captured by carriage mount boss and clevis cheeks (hinge capture).
    ctx.allow_overlap(
        carriage_0,
        finger_0,
        elem_a="carriage_body",
        elem_b="bellows",
        reason="Finger neck is captured by carriage mount boss and clevis cheeks.",
    )
    ctx.allow_overlap(
        carriage_1,
        finger_1,
        elem_a="carriage_body",
        elem_b="bellows",
        reason="Finger neck is captured by carriage mount boss and clevis cheeks.",
    )

    ctx.expect_overlap(
        finger_0,
        finger_1,
        axes="y",
        min_overlap=0.020,
        elem_a="bellows",
        elem_b="bellows",
        name="opposing fingers share gripping width",
    )
    ctx.expect_origin_distance(
        carriage_0,
        carriage_1,
        axes="x",
        min_dist=0.095,
        max_dist=0.115,
        name="carriage span matches compact two-jaw layout at rest",
    )

    # Finger bellows hangs below the carriage main body in Z.
    ctx.expect_gap(
        carriage_0,
        finger_0,
        axis="z",
        max_penetration=0.008,
        negative_elem="bellows",
        name="finger_0 hangs below carriage_0 main body",
    )
    ctx.expect_gap(
        carriage_1,
        finger_1,
        axis="z",
        max_penetration=0.008,
        negative_elem="bellows",
        name="finger_1 hangs below carriage_1 main body",
    )

    # Prismatic span-adjust: carriages move outward symmetrically.
    rest_c0 = ctx.part_world_position(carriage_0)
    rest_c1 = ctx.part_world_position(carriage_1)
    with ctx.pose({slide_0: 0.025}):
        wide_c0 = ctx.part_world_position(carriage_0)
        wide_c1 = ctx.part_world_position(carriage_1)
    ctx.check(
        "prismatic span slide widens jaw gap symmetrically",
        rest_c0 is not None
        and rest_c1 is not None
        and wide_c0 is not None
        and wide_c1 is not None
        and wide_c0[0] < rest_c0[0] - 0.020
        and wide_c1[0] > rest_c1[0] + 0.020,
        details=f"rest_c0={rest_c0}, wide_c0={wide_c0}, rest_c1={rest_c1}, wide_c1={wide_c1}",
    )

    # Bend closure test: fingers close inward at max bend.
    rest_0 = ctx.part_element_world_aabb(finger_0, elem="bellows")
    rest_1 = ctx.part_element_world_aabb(finger_1, elem="bellows")
    with ctx.pose({bend: 0.55}):
        closed_0 = ctx.part_element_world_aabb(finger_0, elem="bellows")
        closed_1 = ctx.part_element_world_aabb(finger_1, elem="bellows")
    ctx.check(
        "positive bend closes both soft fingers inward",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[1][0] > rest_0[1][0] + 0.020
        and closed_1[0][0] < rest_1[0][0] - 0.020,
        details=f"rest_0={rest_0}, closed_0={closed_0}, rest_1={rest_1}, closed_1={closed_1}",
    )

    # Combined slide + bend: fingers still close inward even at widened span.
    with ctx.pose({slide_0: 0.025, bend: 0.55}):
        wide_closed_0 = ctx.part_element_world_aabb(finger_0, elem="bellows")
        wide_closed_1 = ctx.part_element_world_aabb(finger_1, elem="bellows")
    ctx.check(
        "bend closure works at widened span",
        wide_closed_0 is not None
        and wide_closed_1 is not None
        and wide_closed_0[1][0] > rest_0[1][0] + 0.010
        and wide_closed_1[0][0] < rest_1[0][0] - 0.010,
        details=f"wide_closed_0={wide_closed_0}, wide_closed_1={wide_closed_1}",
    )

    rest_hose = ctx.part_world_aabb(hose)
    with ctx.pose({swivel: 0.25}):
        turned_hose = ctx.part_world_aabb(hose)
    ctx.check(
        "hose connector has small swivel motion",
        rest_hose is not None and turned_hose is not None and abs(turned_hose[1][0] - rest_hose[1][0]) > 0.001,
        details=f"rest={rest_hose}, turned={turned_hose}",
    )

    return ctx.report()


object_model = build_object_model()
