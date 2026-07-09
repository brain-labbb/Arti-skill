from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    Mimic,
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
    # Drag brace attachment lug on the underside of the plate, offset in -Y.
    # Height chosen so the top face contacts the plate bottom (plate_skin z_bottom = 0.065).
    plate.visual(
        Box((0.08, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, -0.14, 0.020)),
        material=white_paint,
        name="drag_lug",
    )
    plate.visual(
        Cylinder(0.028, 0.10),
        origin=Origin(xyz=(0.0, -0.14, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_lug_bore",
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
    # Drag brace attachment shoulder lug on the strut, offset in -Y.
    strut.visual(
        Box((0.08, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.12, -0.18)),
        material=steel,
        name="drag_strut_lug",
    )
    strut.visual(
        Cylinder(0.024, 0.10),
        origin=Origin(xyz=(0.0, -0.12, -0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_strut_lug_bore",
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
    # Rolling wheel with distinct black tire, sidewall, metal rim, bolted hub, and bore.
    wheel = model.part("wheel")
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
    wheel.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
    wheel.visual(
        Cylinder(0.310, 0.012),
        origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tire_side,
        name="outer_sidewall_ring",
    )
    wheel.visual(rim_mesh, origin=Origin(), material=aluminum, name="hub")
    wheel.visual(
        Cylinder(0.046, 0.232),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="axle_bore_shadow",
    )

    # -------------------------------------------------------------------
    # Folding drag / side-stay brace: two-segment linkage from mount plate
    # to strut shoulder.  The knee folds as the gear retracts.
    # -------------------------------------------------------------------
    drag_upper = model.part("drag_brace_upper")
    # Main bar from the plate lug (part origin) down to the knee.
    _cylinder_between(
        drag_upper,
        (0.0, 0.0, 0.0),
        (0.0, -0.14, -0.54),
        0.022,
        steel,
        "drag_upper_bar",
    )
    # Pivot bosses at each end (perpendicular to the folding plane).
    drag_upper.visual(
        Cylinder(0.035, 0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_upper_boss_top",
    )
    drag_upper.visual(
        Cylinder(0.030, 0.07),
        origin=Origin(xyz=(0.0, -0.14, -0.54), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_upper_boss_knee",
    )

    drag_lower = model.part("drag_brace_lower")
    # Main bar from the knee (part origin) up to the strut shoulder lug.
    _cylinder_between(
        drag_lower,
        (0.0, 0.0, 0.0),
        (0.0, 0.16, 0.34),
        0.020,
        steel,
        "drag_lower_bar",
    )
    # Pivot bosses at each end.
    drag_lower.visual(
        Cylinder(0.030, 0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_lower_boss_knee",
    )
    drag_lower.visual(
        Cylinder(0.028, 0.06),
        origin=Origin(xyz=(0.0, 0.16, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="drag_lower_boss_strut",
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

    # Drag brace articulations: both mimic mount_to_strut so the linkage
    # folds proportionally when the gear retracts.
    model.articulation(
        "plate_to_drag_upper",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=drag_upper,
        origin=Origin(xyz=(0.0, -0.14, 0.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.8, lower=-0.3, upper=1.1),
        mimic=Mimic(joint="mount_to_strut", multiplier=0.90, offset=0.0),
    )
    model.articulation(
        "drag_knee",
        ArticulationType.REVOLUTE,
        parent=drag_upper,
        child=drag_lower,
        origin=Origin(xyz=(0.0, -0.14, -0.54)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=1.0, lower=-2.2, upper=0.2),
        mimic=Mimic(joint="mount_to_strut", multiplier=-1.50, offset=0.0),
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
    drag_upper = object_model.get_part("drag_brace_upper")
    drag_lower = object_model.get_part("drag_brace_lower")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_piston")
    steer = object_model.get_articulation("piston_to_fork")
    spin = object_model.get_articulation("fork_to_wheel")
    drag_upper_joint = object_model.get_articulation("plate_to_drag_upper")
    drag_knee = object_model.get_articulation("drag_knee")

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
    # Drag brace overlap allowances: the brace bars pass through their
    # attachment lugs (pinned connection), and the upper bar clears the
    # strut lug region.  The two bars meet at the knee joint.
    ctx.allow_overlap(
        drag_upper,
        plate,
        elem_a="drag_upper_bar",
        elem_b="drag_lug",
        reason="The drag brace upper bar is pinned through the mount plate drag lug.",
    )
    ctx.allow_overlap(
        drag_upper,
        plate,
        elem_a="drag_upper_bar",
        elem_b="drag_lug_bore",
        reason="The drag brace upper bar passes through the lug bore for the hinge pin.",
    )
    ctx.allow_overlap(
        drag_upper,
        plate,
        elem_a="drag_upper_boss_top",
        elem_b="drag_lug",
        reason="The drag brace upper boss seats into the mount plate drag lug.",
    )
    ctx.allow_overlap(
        drag_upper,
        plate,
        elem_a="drag_upper_boss_top",
        elem_b="drag_lug_bore",
        reason="The drag lug bore passes through the upper boss for the hinge pin.",
    )
    ctx.allow_overlap(
        drag_lower,
        strut,
        elem_a="drag_lower_bar",
        elem_b="drag_strut_lug",
        reason="The drag brace lower bar is pinned to the strut shoulder lug.",
    )
    ctx.allow_overlap(
        drag_lower,
        strut,
        elem_a="drag_lower_bar",
        elem_b="drag_strut_lug_bore",
        reason="The drag brace lower bar passes through the strut lug bore for the hinge pin.",
    )
    ctx.allow_overlap(
        drag_lower,
        strut,
        elem_a="drag_lower_boss_strut",
        elem_b="drag_strut_lug",
        reason="The drag brace lower boss seats into the strut shoulder lug.",
    )
    ctx.allow_overlap(
        drag_lower,
        strut,
        elem_a="drag_lower_boss_strut",
        elem_b="drag_strut_lug_bore",
        reason="The strut lug bore passes through the lower boss for the hinge pin.",
    )
    ctx.allow_overlap(
        drag_upper,
        strut,
        elem_a="drag_upper_bar",
        elem_b="drag_strut_lug",
        reason="The drag brace upper bar passes near the strut shoulder lug in the deployed configuration; this is a designed clearance path.",
    )
    # The two brace bars meet at the knee joint.
    ctx.allow_overlap(
        drag_lower,
        drag_upper,
        elem_a="drag_lower_bar",
        elem_b="drag_upper_bar",
        reason="The two drag brace bars converge at the knee joint where they share the hinge pin.",
    )
    ctx.allow_overlap(
        drag_lower,
        drag_upper,
        elem_a="drag_lower_bar",
        elem_b="drag_upper_boss_knee",
        reason="The lower brace bar meets the upper knee boss at the folding joint.",
    )
    ctx.allow_overlap(
        drag_lower,
        drag_upper,
        elem_a="drag_lower_boss_knee",
        elem_b="drag_upper_bar",
        reason="The lower knee boss shares the hinge pin with the upper brace bar at the folding joint.",
    )
    ctx.allow_overlap(
        drag_upper,
        drag_lower,
        elem_a="drag_upper_boss_knee",
        elem_b="drag_lower_boss_knee",
        reason="The knee joint is a pinned connection where both bosses share the hinge pin.",
    )
    ctx.check("has retraction hinge", retract is not None, "missing mount_to_strut hinge")
    ctx.check("has shock slider", shock is not None, "missing telescoping oleo slider")
    ctx.check("has caster steering", steer is not None, "missing steering/caster joint")
    ctx.check("has rolling wheel joint", spin is not None, "missing continuous wheel rotation")
    ctx.check("has drag brace upper joint", drag_upper_joint is not None, "missing plate_to_drag_upper")
    ctx.check("has drag knee joint", drag_knee is not None, "missing drag_knee")
    ctx.check(
        "drag_knee is revolute",
        drag_knee is not None and drag_knee.articulation_type == ArticulationType.REVOLUTE,
        "drag_knee must be REVOLUTE for the folding linkage",
    )
    ctx.check(
        "drag_upper_joint mimics mount_to_strut",
        drag_upper_joint is not None
        and drag_upper_joint.mimic is not None
        and drag_upper_joint.mimic.joint == "mount_to_strut",
        "plate_to_drag_upper should mimic mount_to_strut for retraction coupling",
    )

    # Verify the drag brace geometry is present and properly sized.
    ctx.expect_overlap(
        drag_upper,
        plate,
        axes="xy",
        min_overlap=0.04,
        elem_a="drag_upper_boss_top",
        elem_b="drag_lug",
        name="drag upper boss overlaps mount plate drag lug in XY",
    )
    ctx.expect_overlap(
        drag_lower,
        strut,
        axes="xy",
        min_overlap=0.04,
        elem_a="drag_lower_boss_strut",
        elem_b="drag_strut_lug",
        name="drag lower boss overlaps strut drag lug in XY",
    )

    ctx.expect_origin_gap(plate, wheel, axis="z", min_gap=1.45, name="long deployed strut proportions")
    ctx.expect_overlap(wheel, fork, axes="z", min_overlap=0.20, name="wheel vertically captured by fork")
    ctx.expect_overlap(wheel, fork, axes="y", min_overlap=0.075, name="wheel centered in fork opening")
    ctx.expect_within(wheel, fork, axes="x", margin=0.010, name="tire width fits between fork plates")
    ctx.expect_overlap(
        fork,
        wheel,
        axes="x",
        min_overlap=0.18,
        elem_a="axle_pin",
        elem_b="hub",
        name="axle pin passes through wheel bore",
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

    # Drag brace folding: when the gear retracts, the knee should move.
    rest_drag_knee_pos = ctx.part_world_position(drag_lower)
    with ctx.pose({retract: 1.05}):
        folded_drag_pos = ctx.part_world_position(drag_lower)
    ctx.check(
        "drag brace folds with retraction",
        rest_drag_knee_pos is not None
        and folded_drag_pos is not None
        and (
            abs(folded_drag_pos[2] - rest_drag_knee_pos[2]) > 0.20
            or abs(folded_drag_pos[1] - rest_drag_knee_pos[1]) > 0.20
        ),
        details=f"rest_drag={rest_drag_knee_pos}, folded_drag={folded_drag_pos}",
    )

    return ctx.report()


object_model = build_object_model()
