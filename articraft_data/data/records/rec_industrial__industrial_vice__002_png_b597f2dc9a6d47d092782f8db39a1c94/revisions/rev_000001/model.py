from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BLUE_PAINT = "gloss_chipped_blue"
STEEL = "brushed_steel"
DARK = "dark_end_caps"
BARE = "bare_worn_metal"


def _safe_fillet(shape: cq.Workplane, radius: float) -> cq.Workplane:
    """Keep the cast-metal look where CadQuery can fillet the selected model."""
    try:
        return shape.edges().fillet(radius)
    except Exception:
        return shape


def _base_casting() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.110).extrude(0.030)
    for x, y in ((0.145, 0.075), (0.145, -0.075), (-0.145, 0.075), (-0.145, -0.075)):
        base = base.union(cq.Workplane("XY").center(x, y).circle(0.040).extrude(0.030))

    # Raised swivel turntable and a shallow lower ring.
    base = base.union(cq.Workplane("XY").circle(0.088).extrude(0.035).translate((0.0, 0.0, 0.030)))
    base = base.union(cq.Workplane("XY").circle(0.125).extrude(0.010).translate((0.0, 0.0, 0.010)))

    for x, y in ((0.145, 0.075), (0.145, -0.075), (-0.145, 0.075), (-0.145, -0.075)):
        cutter = cq.Workplane("XY").center(x, y).circle(0.014).extrude(0.090).translate((0.0, 0.0, -0.015))
        base = base.cut(cutter)

    return _safe_fillet(base, 0.004)


def _body_casting() -> cq.Workplane:
    width = 0.160
    side_profile = [
        (-0.160, 0.000),
        (0.135, 0.000),
        (0.150, 0.050),
        (0.128, 0.128),
        (0.098, 0.198),
        (0.042, 0.256),
        (-0.095, 0.260),
        (-0.138, 0.224),
        (-0.158, 0.130),
    ]
    body = cq.Workplane("XZ").polyline(side_profile).close().extrude(width).translate((0.0, width / 2.0, 0.0))
    body = body.union(cq.Workplane("XY").circle(0.080).extrude(0.035))

    # Side-visible arched throat and a lower rectangular slide channel.
    throat_rect = (
        cq.Workplane("XZ")
        .center(0.028, 0.125)
        .rect(0.112, 0.128)
        .extrude(width + 0.030)
        .translate((0.0, width / 2.0 + 0.015, 0.0))
    )
    throat_arch = (
        cq.Workplane("XZ")
        .center(0.030, 0.188)
        .circle(0.056)
        .extrude(width + 0.030)
        .translate((0.0, width / 2.0 + 0.015, 0.0))
    )
    slide_tunnel = cq.Workplane("XY").box(0.285, 0.078, 0.052).translate((0.000, 0.000, 0.066))
    body = body.cut(throat_rect).cut(throat_arch).cut(slide_tunnel)

    return _safe_fillet(body, 0.003)


def _front_jaw_casting() -> cq.Workplane:
    width = 0.156
    profile = [
        (-0.055, -0.020),
        (0.045, -0.020),
        (0.064, 0.042),
        (0.058, 0.177),
        (0.032, 0.224),
        (-0.035, 0.224),
        (-0.058, 0.166),
        (-0.052, 0.040),
    ]
    jaw = cq.Workplane("XZ").polyline(profile).close().extrude(width).translate((0.0, width / 2.0, 0.0))
    throat_rect = (
        cq.Workplane("XZ")
        .center(0.005, 0.102)
        .rect(0.062, 0.102)
        .extrude(width + 0.030)
        .translate((0.0, width / 2.0 + 0.015, 0.0))
    )
    throat_arch = (
        cq.Workplane("XZ")
        .center(0.005, 0.150)
        .circle(0.034)
        .extrude(width + 0.030)
        .translate((0.0, width / 2.0 + 0.015, 0.0))
    )
    jaw = jaw.cut(throat_rect).cut(throat_arch)
    return _safe_fillet(jaw, 0.003)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blue_bench_vise")

    blue = model.material(BLUE_PAINT, rgba=(0.005, 0.155, 0.470, 1.0))
    steel = model.material(STEEL, rgba=(0.72, 0.72, 0.69, 1.0))
    dark = model.material(DARK, rgba=(0.015, 0.014, 0.012, 1.0))
    bare = model.material(BARE, rgba=(0.52, 0.54, 0.55, 1.0))

    base = model.part("base", meta={"bolt_holes": 4, "finish": "glossy chipped blue paint"})
    base.visual(
        mesh_from_cadquery(_base_casting(), "base_with_bolt_holes", tolerance=0.0008, angular_tolerance=0.08),
        material=blue,
        name="base_with_bolt_holes",
    )
    # Small exposed-metal chips on the painted casting, seated into the blue surface.
    base.visual(Box((0.042, 0.008, 0.002)), origin=Origin(xyz=(0.045, -0.110, 0.031)), material=bare, name="base_paint_chip_0")
    base.visual(Box((0.030, 0.007, 0.002)), origin=Origin(xyz=(-0.040, 0.095, 0.031)), material=bare, name="base_paint_chip_1")

    body = model.part("body", meta={"cast_vise_body": True, "rear_anvil": True})
    body.visual(
        mesh_from_cadquery(_body_casting(), "body_casting", tolerance=0.0008, angular_tolerance=0.08),
        material=blue,
        name="body_casting",
    )
    body.visual(Box((0.130, 0.128, 0.070)), origin=Origin(xyz=(-0.220, 0.000, 0.150)), material=steel, name="rear_anvil")
    body.visual(Box((0.014, 0.148, 0.064)), origin=Origin(xyz=(-0.145, 0.000, 0.205)), material=steel, name="fixed_plate")
    for z in (0.190, 0.220):
        body.visual(
            Cylinder(radius=0.007, length=0.005),
            origin=Origin(xyz=(-0.137, -0.042, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"fixed_plate_screw_{int(z * 1000)}",
        )
        body.visual(
            Cylinder(radius=0.007, length=0.005),
            origin=Origin(xyz=(-0.137, 0.042, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"fixed_plate_screw_b_{int(z * 1000)}",
        )
    # Raised side letters and ribs are simple cast-on embossing.
    for i, x in enumerate((-0.133, -0.115, -0.096, -0.076)):
        body.visual(Box((0.010, 0.006, 0.027)), origin=Origin(xyz=(x, -0.083, 0.104)), material=blue, name=f"embossed_mark_{i}")
    body.visual(Box((0.014, 0.010, 0.142)), origin=Origin(xyz=(-0.142, -0.083, 0.103)), material=blue, name="rear_rib")
    body.visual(Box((0.012, 0.010, 0.115)), origin=Origin(xyz=(0.108, -0.083, 0.086)), material=blue, name="front_rib")
    body.visual(Box((0.035, 0.008, 0.002)), origin=Origin(xyz=(-0.082, -0.084, 0.244)), material=bare, name="body_paint_chip")

    front_jaw = model.part("front_jaw", meta={"sliding_jaw": True, "arched_throat": True})
    front_jaw.visual(
        mesh_from_cadquery(_front_jaw_casting(), "front_casting", tolerance=0.0008, angular_tolerance=0.08),
        material=blue,
        name="front_casting",
    )
    front_jaw.visual(Box((0.226, 0.045, 0.038)), origin=Origin(xyz=(-0.098, 0.000, 0.000)), material=blue, name="slide_bar")
    front_jaw.visual(Box((0.014, 0.148, 0.064)), origin=Origin(xyz=(-0.050, 0.000, 0.140)), material=steel, name="moving_plate")
    for z in (0.125, 0.155):
        front_jaw.visual(
            Cylinder(radius=0.007, length=0.005),
            origin=Origin(xyz=(-0.059, -0.042, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"moving_plate_screw_{int(z * 1000)}",
        )
        front_jaw.visual(
            Cylinder(radius=0.007, length=0.005),
            origin=Origin(xyz=(-0.059, 0.042, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"moving_plate_screw_b_{int(z * 1000)}",
        )
    front_jaw.visual(Box((0.028, 0.007, 0.002)), origin=Origin(xyz=(0.020, -0.080, 0.198)), material=bare, name="jaw_paint_chip")

    lead_screw = model.part("lead_screw", meta={"spins_to_drive_jaw": True})
    lead_screw.visual(
        Cylinder(radius=0.011, length=0.320),
        origin=Origin(xyz=(0.110, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lead_screw",
    )
    lead_screw.visual(
        Cylinder(radius=0.023, length=0.052),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="screw_collar",
    )
    lead_screw.visual(
        Cylinder(radius=0.0075, length=0.205),
        origin=Origin(xyz=(0.250, 0.000, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="t_handle",
    )
    lead_screw.visual(Sphere(radius=0.017), origin=Origin(xyz=(0.250, 0.106, 0.000)), material=dark, name="handle_knob_0")
    lead_screw.visual(Sphere(radius=0.017), origin=Origin(xyz=(0.250, -0.106, 0.000)), material=dark, name="handle_knob_1")
    lead_screw.visual(Sphere(radius=0.014), origin=Origin(xyz=(0.282, 0.000, 0.000)), material=dark, name="screw_end_cap")

    side_lock = model.part("side_lock", meta={"swivel_lock_handle": True})
    side_lock.visual(Cylinder(radius=0.008, length=0.050), origin=Origin(xyz=(0.000, 0.000, 0.025)), material=steel, name="lock_post")
    side_lock.visual(
        Cylinder(radius=0.005, length=0.075),
        origin=Origin(xyz=(0.000, 0.000, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lock_cross_handle",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "body_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_jaw,
        origin=Origin(xyz=(0.205, 0.0, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=0.0, upper=0.085),
    )
    model.articulation(
        "front_jaw_to_screw",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=lead_screw,
        origin=Origin(xyz=(0.055, 0.0, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "base_to_side_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_lock,
        origin=Origin(xyz=(-0.035, -0.095, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.8, upper=0.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    front_jaw = object_model.get_part("front_jaw")
    lead_screw = object_model.get_part("lead_screw")
    side_lock = object_model.get_part("side_lock")
    slide_joint = object_model.get_articulation("body_to_front_jaw")
    screw_joint = object_model.get_articulation("front_jaw_to_screw")

    ctx.allow_overlap(
        front_jaw,
        lead_screw,
        elem_a="front_casting",
        elem_b="screw_collar",
        reason="The screw collar is intentionally captured in the moving jaw bore.",
    )
    ctx.allow_overlap(
        front_jaw,
        lead_screw,
        elem_a="slide_bar",
        elem_b="lead_screw",
        reason="The lead screw intentionally passes through the sliding bar's threaded bore.",
    )
    ctx.allow_overlap(
        body,
        front_jaw,
        elem_a="body_casting",
        elem_b="slide_bar",
        reason="The sliding bar is intentionally represented as retained inside the cast body's guide tunnel.",
    )
    ctx.allow_overlap(
        front_jaw,
        lead_screw,
        elem_a="front_casting",
        elem_b="lead_screw",
        reason="The lead screw intentionally passes through the moving jaw bore before the handle.",
    )
    ctx.allow_overlap(
        base,
        side_lock,
        elem_a="base_with_bolt_holes",
        elem_b="lock_post",
        reason="The swivel lock post is intentionally seated in a drilled boss on the base.",
    )

    ctx.check(
        "blue cast vise materials",
        all(v.material is not None and getattr(v.material, "name", "") == BLUE_PAINT for v in (base.get_visual("base_with_bolt_holes"), body.get_visual("body_casting"), front_jaw.get_visual("front_casting"))),
        details="Main base, body, and sliding jaw castings should use glossy blue paint.",
    )
    ctx.check("swivel base has four bolt holes", base.meta.get("bolt_holes") == 4 and base.get_visual("base_with_bolt_holes") is not None)
    ctx.check("rear anvil block present", body.meta.get("rear_anvil") is True and body.get_visual("rear_anvil") is not None)
    ctx.check("two jaws with replaceable plates", body.get_visual("fixed_plate") is not None and front_jaw.get_visual("moving_plate") is not None)
    ctx.check("lead screw and T-handle present", lead_screw.get_visual("lead_screw") is not None and lead_screw.get_visual("t_handle") is not None)
    ctx.check("side lock pin handle present", side_lock.get_visual("lock_post") is not None and side_lock.get_visual("lock_cross_handle") is not None)
    ctx.check(
        "sliding jaw joint is non-fixed prismatic",
        slide_joint.articulation_type == ArticulationType.PRISMATIC
        and slide_joint.motion_limits is not None
        and slide_joint.motion_limits.upper > 0.05,
    )
    ctx.check("lead screw spins", screw_joint.articulation_type == ArticulationType.CONTINUOUS)

    ctx.expect_contact(base, body, elem_a="base_with_bolt_holes", elem_b="body_casting", contact_tol=0.0015, name="body seated on swivel base")
    ctx.expect_overlap(body, front_jaw, axes="yz", elem_a="fixed_plate", elem_b="moving_plate", min_overlap=0.050, name="jaw plates stay parallel and aligned")
    ctx.expect_gap(front_jaw, body, axis="x", positive_elem="moving_plate", negative_elem="fixed_plate", min_gap=0.270, max_gap=0.330, name="initial jaw opening")
    ctx.expect_overlap(front_jaw, body, axes="x", elem_a="slide_bar", elem_b="body_casting", min_overlap=0.110, name="slide bar retained in body")
    ctx.expect_within(front_jaw, body, axes="y", inner_elem="slide_bar", outer_elem="body_casting", margin=0.006, name="slide bar centered between body cheeks")
    ctx.expect_overlap(lead_screw, front_jaw, axes="x", elem_a="lead_screw", elem_b="slide_bar", min_overlap=0.006, name="lead screw enters sliding bar bore")
    ctx.expect_overlap(lead_screw, front_jaw, axes="x", elem_a="lead_screw", elem_b="front_casting", min_overlap=0.030, name="lead screw passes through moving jaw bore")
    ctx.expect_overlap(side_lock, base, axes="z", elem_a="lock_post", elem_b="base_with_bolt_holes", min_overlap=0.020, name="lock post inserted into base boss")

    rest_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        extended_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(front_jaw, body, axis="x", positive_elem="moving_plate", negative_elem="fixed_plate", min_gap=0.350, name="jaw opens under prismatic travel")
        ctx.expect_overlap(front_jaw, body, axes="x", elem_a="slide_bar", elem_b="body_casting", min_overlap=0.035, name="extended jaw keeps retained slide engagement")
    ctx.check(
        "front jaw translates outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.070,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
