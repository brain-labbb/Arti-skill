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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder_between(part, p0, p1, radius, material, name):
    """Add a cylinder whose local +Z axis spans two local points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder requested for {name}")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius, length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retractable_landing_gear",
        meta={
            "classification_note": "Reference image appears to show aircraft retractable landing gear despite the Astronomy folder context."
        },
    )

    rubber = Material("mat_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    dark_metal = Material("mat_dark_hardcoat", rgba=(0.05, 0.055, 0.06, 1.0))
    steel = Material("mat_brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    chrome = Material("mat_chrome_piston", rgba=(0.82, 0.86, 0.90, 1.0))
    aluminum = Material("mat_cast_aluminum", rgba=(0.68, 0.69, 0.68, 1.0))
    white_paint = Material("mat_white_painted_plate", rgba=(0.82, 0.84, 0.82, 1.0))
    # Root: compact aircraft-side mounting plate with hinge lugs.
    plate = model.part("mount_plate")
    plate.visual(
        Box((0.66, 0.44, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=white_paint,
        name="plate_skin",
    )
    plate.visual(
        Box((0.11, 0.15, 0.23)),
        origin=Origin(xyz=(-0.265, 0.0, -0.050)),
        material=white_paint,
        name="hinge_lug_0",
    )
    plate.visual(
        Box((0.11, 0.15, 0.23)),
        origin=Origin(xyz=(0.265, 0.0, -0.050)),
        material=white_paint,
        name="hinge_lug_1",
    )
    for x, name in ((-0.265, "lug_bore_0"), (0.265, "lug_bore_1")):
        plate.visual(
            Cylinder(0.045, 0.118),
            origin=Origin(xyz=(x, 0.0, -0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=name,
        )

    # Main retracting strut: long upper shock sleeve, trunnion, and torque links.
    strut = model.part("strut")
    strut.visual(
        Cylinder(0.055, 0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="trunnion_tube",
    )
    strut.visual(
        Box((0.11, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="trunnion_web",
    )
    strut.visual(
        Cylinder(0.075, 0.58),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=dark_metal,
        name="outer_shock_sleeve",
    )
    strut.visual(
        Cylinder(0.088, 0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=steel,
        name="upper_gland_nut",
    )
    strut.visual(
        Cylinder(0.084, 0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.675)),
        material=steel,
        name="lower_sleeve_collar",
    )
    # Scissor/torque links on the visible front side of the oleo.
    _cylinder_between(strut, (-0.050, -0.088, -0.30), (0.050, -0.088, -0.60), 0.014, steel, "torque_link_0")
    _cylinder_between(strut, (0.050, -0.088, -0.30), (-0.050, -0.088, -0.60), 0.014, steel, "torque_link_1")
    for z, name in ((-0.30, "upper_torque_pivot"), (-0.60, "lower_torque_pivot")):
        strut.visual(
            Cylinder(0.020, 0.16),
            origin=Origin(xyz=(0.0, -0.088, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=name,
        )

    # Telescoping chrome piston below the sleeve; the fork/axle rides on this slider.
    piston = model.part("piston")
    piston.visual(
        Cylinder(0.044, 0.47),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=chrome,
        name="chrome_piston",
    )
    piston.visual(
        Cylinder(0.078, 0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.49)),
        material=steel,
        name="lower_oleo_head",
    )
    piston.visual(
        Cylinder(0.063, 0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.565)),
        material=dark_metal,
        name="caster_bearing_stack",
    )

    # Steerable/caster fork: two side plates straddle the tire with a crown bridge and axle bosses.
    fork = model.part("fork")
    fork.visual(
        Box((0.34, 0.10, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=aluminum,
        name="fork_crown",
    )
    fork.visual(
        Cylinder(0.055, 0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="steering_socket",
    )
    for x, name in ((-0.148, "fork_arm_0"), (0.148, "fork_arm_1")):
        fork.visual(
            Box((0.042, 0.080, 0.46)),
            origin=Origin(xyz=(x, 0.0, -0.285)),
            material=aluminum,
            name=name,
        )
        fork.visual(
            Cylinder(0.044, 0.044),
            origin=Origin(xyz=(x, 0.0, -0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{name}_axle_boss",
        )
    fork.visual(
        Cylinder(0.038, 0.265),
        origin=Origin(xyz=(0.0, 0.0, -0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_pin",
    )
    # Solid molded disc wheel: one-piece rounded rubber disc with plain hub face and axle bore.
    wheel = model.part("wheel")

    # Main disc body — solid rounded cylinder with central bore (axis along local X).
    solid_disc_shape = (
        cq.Workplane("YZ")
        .circle(0.315)
        .extrude(0.185)
        .translate((-0.185 / 2.0, 0.0, 0.0))
        .edges("%CIRCLE")
        .fillet(0.015)
        .faces(">X")
        .workplane()
        .hole(0.038)
    )
    solid_disc_mesh = mesh_from_cadquery(solid_disc_shape, "solid_disc_body")
    wheel.visual(solid_disc_mesh, origin=Origin(), material=rubber, name="solid_disc")

    # Plain hub caps — small raised discs on each face (no spokes, no windows).
    hub_outer = (
        cq.Workplane("YZ")
        .workplane(offset=0.185 / 2.0 - 0.001)
        .circle(0.065)
        .extrude(0.006)
    )
    hub_inner = (
        cq.Workplane("YZ")
        .workplane(offset=-0.185 / 2.0 - 0.005)
        .circle(0.065)
        .extrude(0.006)
    )
    hub_cap_shape = hub_outer.union(hub_inner)
    hub_cap_mesh = mesh_from_cadquery(hub_cap_shape, "hub_face")
    wheel.visual(hub_cap_mesh, origin=Origin(), material=aluminum, name="hub_face")

    wheel.visual(
        Cylinder(0.046, 0.232),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_bore_shadow",
    )

    model.articulation(
        "mount_to_strut",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=strut,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15000.0, velocity=0.8, lower=0.0, upper=1.22),
    )
    model.articulation(
        "strut_to_piston",
        ArticulationType.PRISMATIC,
        parent=strut,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, -0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.25, lower=0.0, upper=0.075),
    )
    model.articulation(
        "piston_to_fork",
        ArticulationType.REVOLUTE,
        parent=piston,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, -0.565)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=1.2, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.515)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    ctx.warn("Classification note: reference depicts aircraft retractable landing gear, not an astronomy object.")

    plate = object_model.get_part("mount_plate")
    strut = object_model.get_part("strut")
    piston = object_model.get_part("piston")
    fork = object_model.get_part("fork")
    wheel = object_model.get_part("wheel")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_piston")
    steer = object_model.get_articulation("piston_to_fork")
    spin = object_model.get_articulation("fork_to_wheel")

    ctx.allow_overlap(
        fork,
        piston,
        elem_a="steering_socket",
        elem_b="caster_bearing_stack",
        reason="The fork steering socket is intentionally nested around the caster bearing stack.",
    )
    ctx.allow_overlap(
        fork,
        piston,
        elem_a="steering_socket",
        elem_b="lower_oleo_head",
        reason="The lower oleo head seats into the fork steering socket like a captured caster bearing.",
    )
    ctx.allow_overlap(
        fork,
        piston,
        elem_a="fork_crown",
        elem_b="caster_bearing_stack",
        reason="The vertical caster bearing passes through the fork crown in a compact nose-gear yoke.",
    )
    ctx.allow_overlap(
        piston,
        strut,
        elem_a="chrome_piston",
        elem_b="lower_sleeve_collar",
        reason="The chrome piston is intentionally retained through the lower shock sleeve collar.",
    )
    ctx.allow_overlap(
        piston,
        strut,
        elem_a="chrome_piston",
        elem_b="outer_shock_sleeve",
        reason="The chrome piston telescopes inside the outer shock sleeve.",
    )
    ctx.allow_overlap(
        fork,
        wheel,
        elem_a="axle_pin",
        reason="The wheel hub rotates around a captured axle pin represented inside the hub bore.",
    )
    ctx.allow_overlap(
        fork,
        wheel,
        elem_a="axle_pin",
        elem_b="axle_bore_shadow",
        reason="The captured axle pin intentionally passes through the wheel bore shadow.",
    )
    ctx.allow_overlap(
        fork,
        wheel,
        reason="The only fork/wheel interpenetration is the captured axle passing through the wheel hub.",
    )
    ctx.check("has retraction hinge", retract is not None, "missing mount_to_strut hinge")
    ctx.check("has shock slider", shock is not None, "missing telescoping oleo slider")
    ctx.check("has caster steering", steer is not None, "missing steering/caster joint")
    ctx.check("has rolling wheel joint", spin is not None, "missing continuous wheel rotation")

    # Solid disc wheel variant: the wheel part must have the solid_disc visual
    # (no TireGeometry carcass/tread, no spoked WheelGeometry rim).
    wheel_visual_names = {v.name for v in wheel.visuals}
    ctx.check(
        "wheel has solid molded disc body",
        "solid_disc" in wheel_visual_names,
        "solid_disc visual missing from wheel part",
    )
    ctx.check(
        "wheel has plain hub face caps",
        "hub_face" in wheel_visual_names,
        "hub_face visual missing from wheel part",
    )

    ctx.expect_origin_gap(plate, wheel, axis="z", min_gap=1.45, name="long deployed strut proportions")
    ctx.expect_overlap(wheel, fork, axes="z", min_overlap=0.20, name="wheel vertically captured by fork")
    ctx.expect_overlap(wheel, fork, axes="y", min_overlap=0.075, name="wheel centered in fork opening")
    ctx.expect_within(wheel, fork, axes="x", margin=0.010, name="solid disc width fits between fork plates")
    ctx.expect_overlap(
        fork,
        wheel,
        axes="x",
        min_overlap=0.18,
        elem_a="axle_pin",
        elem_b="solid_disc",
        name="axle pin passes through solid disc wheel",
    )
    ctx.expect_overlap(
        fork,
        wheel,
        axes="x",
        min_overlap=0.18,
        elem_a="axle_pin",
        elem_b="axle_bore_shadow",
        name="axle pin retained in bore shadow",
    )
    ctx.expect_overlap(
        piston,
        strut,
        axes="z",
        min_overlap=0.025,
        elem_a="chrome_piston",
        elem_b="outer_shock_sleeve",
        name="piston retained in shock sleeve",
    )
    ctx.expect_overlap(
        fork,
        piston,
        axes="xy",
        min_overlap=0.06,
        elem_a="steering_socket",
        elem_b="caster_bearing_stack",
        name="caster bearing captured by steering socket",
    )
    rest_wheel = ctx.part_world_position(wheel)
    with ctx.pose({retract: 1.05}):
        folded_wheel = ctx.part_world_position(wheel)
    ctx.check(
        "retraction hinge folds gear upward",
        rest_wheel is not None
        and folded_wheel is not None
        and folded_wheel[2] > rest_wheel[2] + 0.45
        and folded_wheel[1] > rest_wheel[1] + 0.70,
        details=f"rest={rest_wheel}, folded={folded_wheel}",
    )

    rest_fork = ctx.part_world_position(fork)
    with ctx.pose({shock: 0.060}):
        compressed_fork = ctx.part_world_position(fork)
    ctx.check(
        "oleo compression raises fork",
        rest_fork is not None and compressed_fork is not None and compressed_fork[2] > rest_fork[2] + 0.045,
        details=f"rest={rest_fork}, compressed={compressed_fork}",
    )

    return ctx.report()


object_model = build_object_model()
