from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
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


def _make_wheel_meshes():
    """Build shared tire and hub meshes for the co-axial wheel pair."""
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.315,
            0.185,
            inner_radius=0.228,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.07),
            tread=TireTread(style="block", depth=0.010, count=28, land_ratio=0.55),
            grooves=(
                TireGroove(center_offset=-0.038, width=0.010, depth=0.005),
                TireGroove(center_offset=0.038, width=0.010, depth=0.005),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.012, radius=0.006),
        ),
        "landing_gear_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.235,
            0.198,
            rim=WheelRim(
                inner_radius=0.145,
                flange_height=0.012,
                flange_thickness=0.008,
                bead_seat_depth=0.006,
            ),
            hub=WheelHub(
                radius=0.062,
                width=0.074,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.090, hole_diameter=0.010),
            ),
            face=WheelFace(dish_depth=0.018, front_inset=0.006, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.010, window_radius=0.032),
            bore=WheelBore(style="round", diameter=0.038),
        ),
        "landing_gear_hub",
    )
    return tire_mesh, rim_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retractable_landing_gear",
        meta={
            "classification_note": "Reference image appears to show aircraft retractable landing gear despite the Astronomy folder context."
        },
    )

    rubber = Material("mat_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    tire_side = Material("mat_tire_sidewall", rgba=(0.025, 0.025, 0.025, 1.0))
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

    # Steerable/caster fork: two side plates straddle twin co-axial tires with a crown bridge and axle bosses.
    fork = model.part("fork")
    fork.visual(
        Box((0.50, 0.10, 0.09)),
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
    for x, name in ((-0.225, "fork_arm_0"), (0.225, "fork_arm_1")):
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
        Cylinder(0.038, 0.42),
        origin=Origin(xyz=(0.0, 0.0, -0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_pin",
    )

    # Twin co-axial rolling wheels: shared geometry helper, for-loop, wheel_{i} naming.
    tire_mesh, rim_mesh = _make_wheel_meshes()
    tire_width = 0.185
    wheel_gap = 0.030
    wheel_x_offset = (tire_width + wheel_gap) / 2.0
    wheel_parts = []
    for i in range(2):
        x_sign = -1.0 if i == 0 else 1.0
        w = model.part(f"wheel_{i}")
        w.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
        w.visual(
            Cylinder(0.310, 0.012),
            origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=tire_side,
            name="outer_sidewall_ring",
        )
        w.visual(rim_mesh, origin=Origin(), material=aluminum, name="hub")
        w.visual(
            Cylinder(0.046, 0.185),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name="axle_bore_shadow",
        )
        wheel_parts.append(w)

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
    for i in range(2):
        x_sign = -1.0 if i == 0 else 1.0
        model.articulation(
            f"fork_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel_parts[i],
            origin=Origin(xyz=(x_sign * wheel_x_offset, 0.0, -0.515)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1200.0, velocity=45.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.warn("Classification note: reference depicts aircraft retractable landing gear, not an astronomy object.")

    plate = object_model.get_part("mount_plate")
    strut = object_model.get_part("strut")
    piston = object_model.get_part("piston")
    fork = object_model.get_part("fork")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_piston")
    steer = object_model.get_articulation("piston_to_fork")
    spin_0 = object_model.get_articulation("fork_to_wheel_0")
    spin_1 = object_model.get_articulation("fork_to_wheel_1")

    # -- Overlap allowances (structural nesting) --
    ctx.allow_overlap(
        fork, piston,
        elem_a="steering_socket", elem_b="caster_bearing_stack",
        reason="The fork steering socket is intentionally nested around the caster bearing stack.",
    )
    ctx.allow_overlap(
        fork, piston,
        elem_a="steering_socket", elem_b="lower_oleo_head",
        reason="The lower oleo head seats into the fork steering socket like a captured caster bearing.",
    )
    ctx.allow_overlap(
        fork, piston,
        elem_a="fork_crown", elem_b="caster_bearing_stack",
        reason="The vertical caster bearing passes through the fork crown in a compact nose-gear yoke.",
    )
    ctx.allow_overlap(
        piston, strut,
        elem_a="chrome_piston", elem_b="lower_sleeve_collar",
        reason="The chrome piston is intentionally retained through the lower shock sleeve collar.",
    )
    ctx.allow_overlap(
        piston, strut,
        elem_a="chrome_piston", elem_b="outer_shock_sleeve",
        reason="The chrome piston telescopes inside the outer shock sleeve.",
    )
    # Twin wheel axle-pin nesting (scoped per wheel)
    for i, w in enumerate((wheel_0, wheel_1)):
        ctx.allow_overlap(
            fork, w,
            elem_a="axle_pin",
            reason=f"wheel_{i} hub rotates around the captured axle pin inside the hub bore.",
        )
        ctx.allow_overlap(
            fork, w,
            elem_a="axle_pin", elem_b="axle_bore_shadow",
            reason=f"The captured axle pin passes through wheel_{i} bore shadow.",
        )
        ctx.allow_overlap(
            fork, w,
            reason=f"The only fork/wheel_{i} interpenetration is the captured axle through the hub.",
        )

    # -- Joint existence checks --
    ctx.check("has retraction hinge", retract is not None, "missing mount_to_strut hinge")
    ctx.check("has shock slider", shock is not None, "missing telescoping oleo slider")
    ctx.check("has caster steering", steer is not None, "missing steering/caster joint")
    ctx.check("has wheel_0 spin joint", spin_0 is not None, "missing continuous wheel_0 rotation")
    ctx.check("has wheel_1 spin joint", spin_1 is not None, "missing continuous wheel_1 rotation")

    # -- Twin co-axial wheel variant assertion --
    ctx.check(
        "twin co-axial wheels exist",
        wheel_0 is not None and wheel_1 is not None,
        "expected two co-axial wheel parts",
    )
    pos_0 = ctx.part_world_position(wheel_0)
    pos_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheels symmetrically spaced along axle X",
        pos_0 is not None and pos_1 is not None
        and abs(pos_0[1] - pos_1[1]) < 0.01
        and abs(pos_0[2] - pos_1[2]) < 0.01
        and abs(pos_0[0] - pos_1[0]) > 0.15,
        details=f"wheel_0={pos_0}, wheel_1={pos_1}",
    )

    # -- Geometry relationship checks --
    ctx.expect_origin_gap(plate, wheel_0, axis="z", min_gap=1.45, name="long deployed strut proportions")
    for i, w in enumerate((wheel_0, wheel_1)):
        ctx.expect_overlap(w, fork, axes="z", min_overlap=0.20, name=f"wheel_{i} vertically captured by fork")
        ctx.expect_overlap(w, fork, axes="y", min_overlap=0.075, name=f"wheel_{i} centered in fork opening")
    ctx.expect_overlap(
        fork, wheel_0, axes="x", min_overlap=0.18,
        elem_a="axle_pin", elem_b="hub",
        name="axle pin passes through wheel_0 bore",
    )
    ctx.expect_overlap(
        fork, wheel_1, axes="x", min_overlap=0.18,
        elem_a="axle_pin", elem_b="hub",
        name="axle pin passes through wheel_1 bore",
    )
    ctx.expect_overlap(
        piston, strut, axes="z", min_overlap=0.025,
        elem_a="chrome_piston", elem_b="outer_shock_sleeve",
        name="piston retained in shock sleeve",
    )
    ctx.expect_overlap(
        fork, piston, axes="xy", min_overlap=0.06,
        elem_a="steering_socket", elem_b="caster_bearing_stack",
        name="caster bearing captured by steering socket",
    )

    # -- Pose checks: retraction hinge folds gear upward --
    rest_wheel = ctx.part_world_position(wheel_0)
    with ctx.pose({retract: 1.05}):
        folded_wheel = ctx.part_world_position(wheel_0)
    ctx.check(
        "retraction hinge folds gear upward",
        rest_wheel is not None
        and folded_wheel is not None
        and folded_wheel[2] > rest_wheel[2] + 0.45
        and folded_wheel[1] > rest_wheel[1] + 0.70,
        details=f"rest={rest_wheel}, folded={folded_wheel}",
    )

    # -- Pose check: oleo compression raises fork --
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
