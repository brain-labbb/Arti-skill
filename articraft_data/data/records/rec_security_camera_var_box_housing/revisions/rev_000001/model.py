from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cyl_x_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder helper: SDK cylinders are local-Z; rotate them onto world/local X."""

    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder helper: SDK cylinders are local-Z; rotate them onto world/local Y."""

    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _hood_mesh():
    """Connected U-channel sun shade with a buried top rail that grips the rectangular housing."""

    hood = (
        cq.Workplane("XY")
        .box(0.270, 0.108, 0.008)
        .translate((-0.235, 0.0, 0.045))
    )
    # Dropped side cheeks make the visor read as a real open-front hood rather
    # than a flat plank.
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.265, 0.008, 0.040)
        .translate((-0.236, 0.050, 0.024))
    )
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.265, 0.008, 0.040)
        .translate((-0.236, -0.050, 0.024))
    )
    # A shallow underside rail overlaps the rectangular housing at the crown, so
    # the hood is visibly seated and not floating above the camera body.
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.220, 0.030, 0.012)
        .translate((-0.225, 0.0, 0.037))
    )
    return hood


def _front_bezel_mesh():
    """Rectangular black bezel frame around the recessed lens window."""

    bezel = cq.Workplane("XY").box(0.014, 0.082, 0.072)
    bezel = bezel.cut(cq.Workplane("XY").box(0.016, 0.058, 0.048))
    return bezel.translate((-0.345, 0.0, 0.0))


def _retaining_ring_mesh():
    """Annular fixed ring around the pan collar with clearance through its center."""

    return (
        cq.Workplane("YZ")
        .circle(0.050)
        .circle(0.039)
        .extrude(0.004)
        .translate((-0.034, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="light_gray_professional_cctv_camera")

    metal = model.material("light_gray_housing", rgba=(0.78, 0.80, 0.82, 1.0))
    darker_metal = model.material("shadow_gray_metal", rgba=(0.50, 0.53, 0.55, 1.0))
    dark = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    glass = model.material("smoked_lens_glass", rgba=(0.025, 0.035, 0.045, 0.82))
    red = model.material("red_led", rgba=(1.0, 0.04, 0.02, 1.0))

    # Root: a round wall plate with a raised center boss and four visible screw
    # holes on the vertical face.  The plate lies in the YZ plane and the camera
    # projects along negative X.
    plate = model.part("wall_plate")
    plate.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=_cyl_x_origin((0.0, 0.0, 0.0)),
        material=metal,
        name="plate_disk",
    )
    plate.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=_cyl_x_origin((-0.020, 0.0, 0.0)),
        material=metal,
        name="central_boss",
    )
    plate.visual(
        Cylinder(radius=0.058, length=0.004),
        origin=_cyl_x_origin((-0.012, 0.0, 0.0)),
        material=metal,
        name="raised_outer_ring",
    )
    plate.visual(
        mesh_from_cadquery(_retaining_ring_mesh(), "boss_inner_ring", tolerance=0.0007),
        material=darker_metal,
        name="boss_inner_ring",
    )

    screw_positions = (
        (0.048, 0.048),
        (-0.048, 0.048),
        (0.048, -0.048),
        (-0.048, -0.048),
    )
    for idx, (y, z) in enumerate(screw_positions):
        plate.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=_cyl_x_origin((-0.0092, y, z)),
            material=dark,
            name=f"screw_hole_{idx}",
        )
        plate.visual(
            Box((0.003, 0.016, 0.0024)),
            origin=Origin(xyz=(-0.0106, y, z)),
            material=dark,
            name=f"screw_slot_wide_{idx}",
        )
        plate.visual(
            Box((0.003, 0.0024, 0.016)),
            origin=Origin(xyz=(-0.0108, y, z)),
            material=dark,
            name=f"screw_slot_tall_{idx}",
        )

    # First moving bracket stage: a pan swivel collar, tapered-looking neck, and
    # exposed ball at the camera pivot.
    pan = model.part("pan_knuckle")
    pan.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=_cyl_x_origin((-0.009, 0.0, 0.0)),
        material=metal,
        name="pan_collar",
    )
    pan.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=_cyl_x_origin((-0.024, 0.0, 0.0)),
        material=metal,
        name="collar_step",
    )
    pan.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=_cyl_x_origin((-0.056, 0.0, 0.0)),
        material=metal,
        name="stem",
    )
    pan.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(-0.090, 0.0, 0.0)),
        material=darker_metal,
        name="ball",
    )

    # Camera body: rectangular professional housing with flat front face,
    # rectangular lens window, sun shade hood, yoke cheeks, and tilt pin.
    camera = model.part("camera_body")
    camera.visual(
        Box((0.250, 0.088, 0.078)),
        origin=Origin(xyz=(-0.215, 0.0, 0.0)),
        material=metal,
        name="body_housing",
    )
    camera.visual(
        Box((0.012, 0.092, 0.082)),
        origin=Origin(xyz=(-0.087, 0.0, 0.0)),
        material=metal,
        name="rear_plate",
    )
    camera.visual(
        mesh_from_cadquery(_hood_mesh(), "sunshade_hood", tolerance=0.0008),
        material=metal,
        name="sunshade_hood",
    )
    camera.visual(
        mesh_from_cadquery(_front_bezel_mesh(), "front_bezel", tolerance=0.0006),
        material=dark,
        name="front_bezel",
    )
    camera.visual(
        Box((0.013, 0.062, 0.052)),
        origin=Origin(xyz=(-0.345, 0.0, 0.0)),
        material=dark,
        name="recess_back",
    )
    camera.visual(
        Box((0.004, 0.054, 0.044)),
        origin=Origin(xyz=(-0.352, 0.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(-0.355, 0.0, 0.0)),
        material=red,
        name="red_led",
    )
    camera.visual(
        Box((0.094, 0.010, 0.038)),
        origin=Origin(xyz=(-0.048, 0.037, 0.0)),
        material=metal,
        name="yoke_cheek_0",
    )
    camera.visual(
        Box((0.094, 0.010, 0.038)),
        origin=Origin(xyz=(-0.048, -0.037, 0.0)),
        material=metal,
        name="yoke_cheek_1",
    )
    camera.visual(
        Cylinder(radius=0.0065, length=0.090),
        origin=_cyl_y_origin((0.0, 0.0, 0.0)),
        material=darker_metal,
        name="tilt_pin",
    )
    camera.visual(
        Box((0.135, 0.008, 0.008)),
        origin=Origin(xyz=(-0.220, 0.046, -0.015)),
        material=metal,
        name="side_rib_0",
    )
    camera.visual(
        Box((0.135, 0.008, 0.008)),
        origin=Origin(xyz=(-0.220, -0.046, -0.015)),
        material=metal,
        name="side_rib_1",
    )

    model.articulation(
        "plate_to_pan",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=pan,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=camera,
        origin=Origin(xyz=(-0.090, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.65, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("wall_plate")
    pan = object_model.get_part("pan_knuckle")
    camera = object_model.get_part("camera_body")
    pan_joint = object_model.get_articulation("plate_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    ctx.allow_overlap(
        pan,
        camera,
        elem_a="ball",
        elem_b="tilt_pin",
        reason=(
            "The dark tilt pin is intentionally captured through the ball/pivot "
            "knuckle to show the real tilting support."
        ),
    )
    ctx.expect_within(
        camera,
        pan,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="ball",
        margin=0.001,
        name="tilt pin is centered through the ball on non-pin axes",
    )
    ctx.expect_overlap(
        camera,
        pan,
        axes="x",
        elem_a="tilt_pin",
        elem_b="ball",
        min_overlap=0.010,
        name="tilt pin remains captured by ball",
    )

    ctx.expect_gap(
        plate,
        pan,
        axis="x",
        positive_elem="central_boss",
        negative_elem="pan_collar",
        max_gap=0.0015,
        max_penetration=0.0,
        name="pan collar seats against wall-plate boss",
    )
    ctx.expect_overlap(
        camera,
        camera,
        axes="yz",
        elem_a="lens_glass",
        elem_b="front_bezel",
        min_overlap=0.020,
        name="rectangular lens window sits within bezel opening",
    )

    # Variant-specific: verify rectangular housing has box proportions
    # (not cylindrical) with width > height for professional housing silhouette.
    housing_aabb = ctx.part_element_world_aabb(camera, elem="body_housing")
    housing_lo, housing_hi = housing_aabb
    housing_dx = housing_hi[0] - housing_lo[0]
    housing_dy = housing_hi[1] - housing_lo[1]
    housing_dz = housing_hi[2] - housing_lo[2]
    ctx.check(
        "body_housing is rectangular prism (length > width > height)",
        housing_dx > 0.20 and housing_dy > 0.06 and housing_dz > 0.05 and housing_dy > housing_dz,
        details=f"body_housing dims=({housing_dx:.4f}, {housing_dy:.4f}, {housing_dz:.4f})",
    )

    def _center_from_aabb(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="lens_glass"))
    rest_side_rib = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="side_rib_0"))
    with ctx.pose({pan_joint: 0.65}):
        panned_side_rib = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="side_rib_0"))
    with ctx.pose({tilt_joint: 0.55}):
        tilted_lens = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="lens_glass"))

    ctx.check(
        "pan joint rotates around wall-plate normal",
        abs(panned_side_rib[2] - rest_side_rib[2]) > 0.02,
        details=f"rest side rib={rest_side_rib}, panned side rib={panned_side_rib}",
    )
    ctx.check(
        "tilt joint raises camera aim",
        tilted_lens[2] > rest_lens[2] + 0.08,
        details=f"rest lens={rest_lens}, tilted lens={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
