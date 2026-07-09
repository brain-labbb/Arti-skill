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
        # Clevis-style cheeks flank the top of each soft finger, so the hinge/neck
        # reads as captured instead of floating below the actuator.
        shape = shape.union(_cq_box((0.028, 0.005, 0.020), (sx * 0.052, 0.028, 0.094), fillet=0.0015))
        shape = shape.union(_cq_box((0.028, 0.005, 0.020), (sx * 0.052, -0.028, 0.094), fillet=0.0015))

    return shape


def _proximal_finger_shape(inward_sign: float):
    """Proximal segment of a soft pneumatic finger: base pad, upper body, 3 bellows fins, bottom connection pad."""
    # Base neck pad seats into the clevis cheeks of the actuator block.
    shape = _cq_box((0.026, 0.030, 0.012), (0.0, 0.0, -0.006), fillet=0.003)
    # Upper body extends down to the knuckle boundary.
    shape = shape.union(_cq_box((0.016, 0.027, 0.040), (-inward_sign * 0.004, 0.0, -0.028), fillet=0.004))
    # Three proximal bellows fins protrude toward the gripping gap.
    for i in range(3):
        z = -0.018 - i * 0.013
        shape = shape.union(_cq_box((0.031, 0.036, 0.009), (inward_sign * 0.006, 0.0, z), fillet=0.0045))
    # Bottom connection pad bridges to the distal knuckle joint.
    shape = shape.union(_cq_box((0.018, 0.026, 0.008), (0.0, 0.0, -0.050), fillet=0.004))
    return shape


def _distal_finger_shape(inward_sign: float):
    """Distal segment of a soft pneumatic finger: top connection stub, lower body, 2 bellows fins, rounded tip."""
    # Top stub nests into the proximal connection pad at the knuckle.
    shape = _cq_box((0.018, 0.026, 0.010), (0.0, 0.0, -0.005), fillet=0.004)
    # Lower body carries the remaining fins.
    shape = shape.union(_cq_box((0.016, 0.027, 0.028), (-inward_sign * 0.004, 0.0, -0.021), fillet=0.004))
    # Two distal bellows fins continue the ribbed PneuNet pattern.
    for i in range(2):
        z = -0.012 - i * 0.013
        shape = shape.union(_cq_box((0.031, 0.036, 0.009), (inward_sign * 0.006, 0.0, z), fillet=0.0045))
    # Rounded tip cap closes the distal chamber.
    shape = shape.union(_cq_box((0.020, 0.028, 0.014), (inward_sign * 0.001, 0.0, -0.037), fillet=0.005))
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

    # --- Proximal finger segments (carry upper 3 bellows fins) ---
    finger_0 = model.part("finger_0")
    finger_0.visual(
        mesh_from_cadquery(_proximal_finger_shape(1.0), "finger_0_bellows"),
        material=silicone,
        name="bellows",
    )

    finger_1 = model.part("finger_1")
    finger_1.visual(
        mesh_from_cadquery(_proximal_finger_shape(-1.0), "finger_1_bellows"),
        material=silicone,
        name="bellows",
    )

    # --- Distal finger tip segments (carry lower 2 bellows fins + tip cap) ---
    finger_0_tip = model.part("finger_0_tip")
    finger_0_tip.visual(
        mesh_from_cadquery(_distal_finger_shape(1.0), "finger_0_tip_bellows"),
        material=silicone,
        name="bellows",
    )

    finger_1_tip = model.part("finger_1_tip")
    finger_1_tip.visual(
        mesh_from_cadquery(_distal_finger_shape(-1.0), "finger_1_tip_bellows"),
        material=silicone,
        name="bellows",
    )

    # --- Hose swivel connector ---
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

    # --- Articulations ---

    # Proximal bend joints: manifold → proximal finger (same as parent baseline)
    finger_limits = MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.55)
    model.articulation(
        "finger_0_bend",
        ArticulationType.REVOLUTE,
        parent=manifold,
        child=finger_0,
        origin=Origin(xyz=(-0.052, 0.0, 0.083)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=finger_limits,
    )
    model.articulation(
        "finger_1_bend",
        ArticulationType.REVOLUTE,
        parent=manifold,
        child=finger_1,
        origin=Origin(xyz=(0.052, 0.0, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=finger_limits,
        mimic=Mimic(joint="finger_0_bend", multiplier=1.0, offset=0.0),
    )

    # Serial knuckle joints: proximal finger → distal tip (progressive curl)
    # Articulation origins are in the parent's local frame; the knuckle sits at
    # the bottom of the proximal segment (local z ≈ -0.052 from the finger origin).
    knuckle_limits = MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=0.50)
    model.articulation(
        "finger_0_knuckle",
        ArticulationType.REVOLUTE,
        parent=finger_0,
        child=finger_0_tip,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=knuckle_limits,
    )
    model.articulation(
        "finger_1_knuckle",
        ArticulationType.REVOLUTE,
        parent=finger_1,
        child=finger_1_tip,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=knuckle_limits,
        mimic=Mimic(joint="finger_0_knuckle", multiplier=1.0, offset=0.0),
    )

    # Hose swivel connector
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
    finger_0 = object_model.get_part("finger_0")
    finger_1 = object_model.get_part("finger_1")
    finger_0_tip = object_model.get_part("finger_0_tip")
    finger_1_tip = object_model.get_part("finger_1_tip")
    hose = object_model.get_part("hose_swivel")
    bend = object_model.get_articulation("finger_0_bend")
    knuckle = object_model.get_articulation("finger_0_knuckle")
    swivel = object_model.get_articulation("hose_swivel")

    # --- Existence and structure ---
    ctx.check(
        "multi-segment gripper has proximal and distal finger segments",
        all(p is not None for p in (manifold, finger_0, finger_1, finger_0_tip, finger_1_tip, hose)),
        details="Expected manifold, two proximal fingers, two distal tips, and hose swivel.",
    )
    ctx.check(
        "serial knuckle joints exist for progressive curl",
        all(j is not None for j in (
            object_model.get_articulation("finger_0_bend"),
            object_model.get_articulation("finger_0_knuckle"),
            object_model.get_articulation("finger_1_bend"),
            object_model.get_articulation("finger_1_knuckle"),
        )),
        details="Expected proximal bend and serial knuckle joints on both fingers.",
    )

    # --- Spacing and seating ---
    ctx.expect_overlap(
        finger_0, finger_1,
        axes="y", min_overlap=0.020,
        elem_a="bellows", elem_b="bellows",
        name="opposing proximal fingers share gripping width",
    )
    ctx.expect_origin_distance(
        finger_0, finger_1,
        axes="x", min_dist=0.095, max_dist=0.110,
        name="finger spacing matches compact two jaw layout",
    )
    ctx.expect_gap(
        manifold, finger_0,
        axis="z", max_gap=0.0015, max_penetration=0.0005,
        negative_elem="bellows",
        name="finger 0 proximal seats under actuator base",
    )
    ctx.expect_gap(
        manifold, finger_1,
        axis="z", max_gap=0.0015, max_penetration=0.0005,
        negative_elem="bellows",
        name="finger 1 proximal seats under actuator base",
    )

    # --- Knuckle connection pad nesting (intentional local overlap) ---
    ctx.allow_overlap(
        finger_0, finger_0_tip,
        elem_a="bellows", elem_b="bellows",
        reason="Proximal bottom pad and distal top stub nest at the knuckle joint for structural continuity.",
    )
    ctx.allow_overlap(
        finger_1, finger_1_tip,
        elem_a="bellows", elem_b="bellows",
        reason="Proximal bottom pad and distal top stub nest at the knuckle joint for structural continuity.",
    )
    ctx.expect_contact(
        finger_0, finger_0_tip,
        elem_a="bellows", elem_b="bellows",
        contact_tol=0.004,
        name="finger_0 knuckle connection pads are seated",
    )
    ctx.expect_contact(
        finger_1, finger_1_tip,
        elem_a="bellows", elem_b="bellows",
        contact_tol=0.004,
        name="finger_1 knuckle connection pads are seated",
    )

    # --- Progressive curl: knuckle alone moves distal tip inward ---
    rest_tip_0 = ctx.part_element_world_aabb(finger_0_tip, elem="bellows")
    with ctx.pose({knuckle: 0.45}):
        curled_tip_0 = ctx.part_element_world_aabb(finger_0_tip, elem="bellows")
    ctx.check(
        "finger_0_knuckle curls distal tip inward for progressive grip",
        rest_tip_0 is not None and curled_tip_0 is not None
        and curled_tip_0[1][0] > rest_tip_0[1][0] + 0.008,
        details=f"rest={rest_tip_0}, curled={curled_tip_0}",
    )

    # --- Combined proximal bend + knuckle: deeper progressive curl ---
    rest_tip_0 = ctx.part_element_world_aabb(finger_0_tip, elem="bellows")
    with ctx.pose({bend: 0.55, knuckle: 0.45}):
        full_curl_tip_0 = ctx.part_element_world_aabb(finger_0_tip, elem="bellows")
    ctx.check(
        "combined bend+knuckle produces deeper progressive curl",
        rest_tip_0 is not None and full_curl_tip_0 is not None
        and full_curl_tip_0[1][0] > rest_tip_0[1][0] + 0.030,
        details=f"rest={rest_tip_0}, full_curl={full_curl_tip_0}",
    )

    # --- Hose swivel still works ---
    rest_hose = ctx.part_world_aabb(hose)
    with ctx.pose({swivel: 0.25}):
        turned_hose = ctx.part_world_aabb(hose)
    ctx.check(
        "hose connector has small swivel motion",
        rest_hose is not None and turned_hose is not None
        and abs(turned_hose[1][0] - rest_hose[1][0]) > 0.001,
        details=f"rest={rest_hose}, turned={turned_hose}",
    )

    return ctx.report()


object_model = build_object_model()
