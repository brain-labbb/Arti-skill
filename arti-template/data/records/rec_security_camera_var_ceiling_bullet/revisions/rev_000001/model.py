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
    """Connected U-channel sun shade with a buried top rail that grips the camera body."""

    hood = (
        cq.Workplane("XY")
        .box(0.270, 0.124, 0.008)
        .translate((-0.235, 0.0, 0.054))
    )
    # Dropped side cheeks make the visor read as a real open-front hood rather
    # than a flat plank.
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.265, 0.008, 0.055)
        .translate((-0.236, 0.058, 0.026))
    )
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.265, 0.008, 0.055)
        .translate((-0.236, -0.058, 0.026))
    )
    # A shallow underside rail overlaps the cylindrical housing at the crown, so
    # the hood is visibly seated and not floating above the camera barrel.
    hood = hood.union(
        cq.Workplane("XY")
        .box(0.220, 0.030, 0.010)
        .translate((-0.225, 0.0, 0.045))
    )
    return hood


def _front_bezel_mesh():
    """Thin black annular front trim around the recessed lens."""

    return (
        cq.Workplane("YZ")
        .circle(0.050)
        .circle(0.032)
        .extrude(0.014)
        .translate((-0.354, 0.0, 0.0))
    )


def _boss_ring_mesh():
    """Annular ring below the ceiling-plate central boss (horizontal orientation)."""

    return (
        cq.Workplane("XY")
        .circle(0.050)
        .circle(0.039)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.044))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_ceiling_pendant_cctv_camera")

    # White indoor-pendant colorway
    white = model.material("white_paint", rgba=(0.92, 0.92, 0.90, 1.0))
    light_gray = model.material("light_gray_metal", rgba=(0.78, 0.78, 0.76, 1.0))
    dark = model.material("matte_black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass = model.material("smoked_lens_glass", rgba=(0.025, 0.035, 0.045, 0.82))
    red = model.material("red_led", rgba=(1.0, 0.04, 0.02, 1.0))

    # ── Root: circular ceiling plate ──────────────────────────────────────
    # Horizontal disk (XY plane) flush against the ceiling at Z=0, with a
    # central boss hanging below and four visible screw holes on the bottom
    # face.  The camera assembly hangs below along -Z.
    plate = model.part("ceiling_plate")
    plate.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=white,
        name="plate_disk",
    )
    plate.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=white,
        name="central_boss",
    )
    plate.visual(
        Cylinder(radius=0.058, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=white,
        name="raised_outer_ring",
    )
    plate.visual(
        mesh_from_cadquery(_boss_ring_mesh(), "boss_inner_ring", tolerance=0.0007),
        material=light_gray,
        name="boss_inner_ring",
    )

    screw_positions = (
        (0.048, 0.048),
        (-0.048, 0.048),
        (0.048, -0.048),
        (-0.048, -0.048),
    )
    for idx, (sx, sy) in enumerate(screw_positions):
        plate.visual(
            Cylinder(radius=0.0075, length=0.003),
            origin=Origin(xyz=(sx, sy, -0.0185)),
            material=dark,
            name=f"screw_hole_{idx}",
        )
        plate.visual(
            Box((0.016, 0.0024, 0.003)),
            origin=Origin(xyz=(sx, sy, -0.0185)),
            material=dark,
            name=f"screw_slot_wide_{idx}",
        )
        plate.visual(
            Box((0.0024, 0.016, 0.003)),
            origin=Origin(xyz=(sx, sy, -0.0185)),
            material=dark,
            name=f"screw_slot_tall_{idx}",
        )

    # ── Pan knuckle: vertical drop stem ───────────────────────────────────
    # Pan collar seats against the bottom of the central boss, then a stepped
    # neck and long vertical stem ending in the exposed ball pivot.
    # Coordinates are in pan part frame (origin at plate_to_pan joint).
    pan = model.part("pan_knuckle")
    pan.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=white,
        name="pan_collar",
    )
    pan.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=white,
        name="collar_step",
    )
    pan.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=white,
        name="stem",
    )
    pan.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=light_gray,
        name="ball",
    )

    # ── Camera body: bullet cylinder hanging from the ball pivot ──────────
    # Identical geometry to the parent baseline; extends along -X from the
    # ball pivot in camera part frame.
    camera = model.part("camera_body")
    camera.visual(
        Cylinder(radius=0.045, length=0.250),
        origin=_cyl_x_origin((-0.215, 0.0, 0.0)),
        material=white,
        name="body_cylinder",
    )
    camera.visual(
        Cylinder(radius=0.047, length=0.028),
        origin=_cyl_x_origin((-0.105, 0.0, 0.0)),
        material=white,
        name="rear_collar",
    )
    camera.visual(
        mesh_from_cadquery(_hood_mesh(), "sunshade_hood", tolerance=0.0008),
        material=white,
        name="sunshade_hood",
    )
    camera.visual(
        mesh_from_cadquery(_front_bezel_mesh(), "front_bezel", tolerance=0.0006),
        material=dark,
        name="front_bezel",
    )
    camera.visual(
        Cylinder(radius=0.033, length=0.013),
        origin=_cyl_x_origin((-0.347, 0.0, 0.0)),
        material=dark,
        name="recess_back",
    )
    camera.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=_cyl_x_origin((-0.354, 0.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(-0.357, 0.0, 0.0)),
        material=red,
        name="red_led",
    )
    camera.visual(
        Box((0.094, 0.010, 0.038)),
        origin=Origin(xyz=(-0.048, 0.039, 0.0)),
        material=white,
        name="yoke_cheek_0",
    )
    camera.visual(
        Box((0.094, 0.010, 0.038)),
        origin=Origin(xyz=(-0.048, -0.039, 0.0)),
        material=white,
        name="yoke_cheek_1",
    )
    camera.visual(
        Cylinder(radius=0.0065, length=0.090),
        origin=_cyl_y_origin((0.0, 0.0, 0.0)),
        material=light_gray,
        name="tilt_pin",
    )
    camera.visual(
        Box((0.135, 0.007, 0.007)),
        origin=Origin(xyz=(-0.220, 0.040, -0.026)),
        material=white,
        name="side_rib_0",
    )
    camera.visual(
        Box((0.135, 0.007, 0.007)),
        origin=Origin(xyz=(-0.220, -0.040, -0.026)),
        material=white,
        name="side_rib_1",
    )

    # ── Articulations ─────────────────────────────────────────────────────
    # Pan: rotation about vertical Z axis so the camera sweeps horizontally.
    model.articulation(
        "plate_to_pan",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-1.15, upper=1.15),
    )
    # Tilt: rotation about horizontal Y axis at the ball pivot.
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.2, lower=-0.65, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("ceiling_plate")
    pan = object_model.get_part("pan_knuckle")
    camera = object_model.get_part("camera_body")
    pan_joint = object_model.get_articulation("plate_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")

    # Tilt pin captured through the ball pivot.
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

    # Ceiling-plate boss seats against pan collar along vertical Z axis.
    ctx.expect_gap(
        plate,
        pan,
        axis="z",
        positive_elem="central_boss",
        negative_elem="pan_collar",
        max_gap=0.0015,
        max_penetration=0.0,
        name="pan collar seats against ceiling-plate boss along vertical drop axis",
    )

    ctx.expect_overlap(
        camera,
        camera,
        axes="yz",
        elem_a="lens_glass",
        elem_b="front_bezel",
        min_overlap=0.020,
        name="dark lens sits within front bezel opening",
    )

    def _center_from_aabb(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_lens = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="lens_glass"))
    rest_body = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="body_cylinder"))

    with ctx.pose({pan_joint: 0.65}):
        panned_body = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="body_cylinder"))
    with ctx.pose({tilt_joint: 0.55}):
        tilted_lens = _center_from_aabb(ctx.part_element_world_aabb(camera, elem="lens_glass"))

    # Pan joint rotates camera around vertical stem axis (horizontal sweep).
    ctx.check(
        "ceiling_plate pan joint rotates camera around vertical Z axis",
        abs(panned_body[1] - rest_body[1]) > 0.02 or abs(panned_body[0] - rest_body[0]) > 0.02,
        details=f"rest body={rest_body}, panned body={panned_body}",
    )
    # Tilt joint raises camera aim upward.
    ctx.check(
        "tilt joint raises camera aim",
        tilted_lens[2] > rest_lens[2] + 0.04,
        details=f"rest lens={rest_lens}, tilted lens={tilted_lens}",
    )

    # Pendant mount: camera hangs below the ceiling plate (all below Z=0).
    ctx.check(
        "camera body hangs below ceiling plate",
        rest_body[2] < -0.05,
        details=f"body center Z={rest_body[2]}, expected well below ceiling",
    )

    return ctx.report()


object_model = build_object_model()
