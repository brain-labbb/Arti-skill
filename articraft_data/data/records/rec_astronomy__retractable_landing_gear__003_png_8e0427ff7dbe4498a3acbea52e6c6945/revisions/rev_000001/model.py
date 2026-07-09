from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _axis_origin_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    """Origin/length for a cylinder whose local +Z runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    radial = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx) if radial > 1e-9 else 0.0
    pitch = math.atan2(radial, dz)
    return (
        Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _plate_with_bolt_holes() -> object:
    """Black square mounting plate with four through holes, like the reference."""
    plate = cq.Workplane("XY").box(0.34, 0.024, 0.28)
    hole_points = [(-0.135, -0.105), (0.135, -0.105), (-0.135, 0.105), (0.135, 0.105)]
    return (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(0.020)
        .edges("|Y")
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retractable_landing_gear",
        meta={
            "classification_note": (
                "Folder context says Astronomy, but the reference image and prompt depict "
                "a retractable landing gear assembly; the model follows the visible landing gear."
            )
        },
    )

    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    dark = model.material("black_rubber", rgba=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.60, 1.0))
    polished = model.material("polished_chrome", rgba=(0.92, 0.92, 0.88, 1.0))
    aluminium = model.material("machined_aluminium", rgba=(0.82, 0.82, 0.78, 1.0))
    brass = model.material("oil_bronze_bushing", rgba=(0.78, 0.58, 0.23, 1.0))

    # Root: compact square mounting plate and upper hinge/side-link lugs.
    mount = model.part("mount_plate")
    mount.visual(
        mesh_from_cadquery(_plate_with_bolt_holes(), "mount_plate_with_holes"),
        origin=Origin(xyz=(0.0, -0.030, 0.080)),
        material=black,
        name="bolt_plate",
    )
    mount.visual(
        Box((0.070, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.000, 0.052)),
        material=black,
        name="center_hinge_block",
    )
    for i, x in enumerate((-0.052, 0.052)):
        mount.visual(
            Box((0.024, 0.062, 0.092)),
            origin=Origin(xyz=(x, 0.003, -0.005)),
            material=black,
            name=f"main_hinge_lug_{i}",
        )
    mount.visual(
        Cylinder(radius=0.009, length=0.145),
        origin=Origin(xyz=(0.0, 0.004, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    mount.visual(
        Box((0.190, 0.030, 0.026)),
        origin=Origin(xyz=(0.080, -0.050, -0.073)),
        material=black,
        name="side_link_ledge",
    )
    for i, x in enumerate((0.102, 0.158)):
        mount.visual(
            Box((0.014, 0.050, 0.048)),
            origin=Origin(xyz=(x, 0.000, -0.030)),
            material=black,
            name=f"brace_upper_lug_{i}",
        )
    mount.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(0.130, 0.000, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="brace_upper_pin",
    )

    # Main folding strut: a long vertical oleo-style sleeve on the retraction hinge.
    main = model.part("main_strut")
    main.visual(
        Cylinder(radius=0.020, length=0.072),
        origin=Origin(xyz=(0.0, 0.004, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle",
    )
    main.visual(
        Cylinder(radius=0.012, length=0.060),
        origin=Origin(xyz=(0.0, 0.022, -0.045)),
        material=steel,
        name="hinge_stem",
    )
    main.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(xyz=(0.0, 0.000, -0.090)),
        material=black,
        name="upper_collar",
    )
    main.visual(
        Cylinder(radius=0.026, length=0.450),
        origin=Origin(xyz=(0.0, 0.000, -0.275)),
        material=steel,
        name="outer_tube",
    )
    main.visual(
        Cylinder(radius=0.029, length=0.035),
        origin=Origin(xyz=(0.0, 0.000, -0.515)),
        material=steel,
        name="lower_collar",
    )
    main.visual(
        Box((0.012, 0.026, 0.170)),
        origin=Origin(xyz=(-0.026, -0.002, -0.235)),
        material=black,
        name="rear_service_pipe",
    )

    # Visible folding drag link. It is a separate revolute link, mimicked from
    # the strut retraction hinge, so the side brace visibly folds with the gear.
    brace = model.part("folding_brace")
    rod_origin, rod_len = _axis_origin_between((-0.003, 0.0, -0.020), (-0.095, 0.0, -0.495))
    brace.visual(
        Cylinder(radius=0.0075, length=rod_len),
        origin=rod_origin,
        material=black,
        name="brace_rod",
    )
    brace.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_eye",
    )
    brace.visual(
        Box((0.024, 0.014, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, -0.026)),
        material=steel,
        name="upper_eye_web",
    )
    brace.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(-0.095, 0.0, -0.495), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lower_eye",
    )

    # Lower telescoping shock piston and fork mount. Positive prismatic motion
    # compresses upward into the outer sleeve.
    slider = model.part("lower_slider")
    slider.visual(
        Cylinder(radius=0.0175, length=0.350),
        origin=Origin(xyz=(0.0, 0.000, -0.112)),
        material=polished,
        name="piston",
    )
    slider.visual(
        Cylinder(radius=0.023, length=0.052),
        origin=Origin(xyz=(0.0, 0.000, -0.284)),
        material=steel,
        name="fork_collar",
    )
    slider.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.0, 0.000, -0.360)),
        material=brass,
        name="lower_bushing",
    )

    # Castering wheel fork: two cheeks straddling the tire, a top bridge, and a
    # real axle pin through the wheel hub.
    fork = model.part("fork")
    fork.visual(
        Box((0.116, 0.034, 0.028)),
        origin=Origin(xyz=(0.0, 0.000, -0.034)),
        material=aluminium,
        name="fork_bridge",
    )
    for i, x in enumerate((-0.050, 0.050)):
        fork.visual(
            Box((0.014, 0.030, 0.228)),
            origin=Origin(xyz=(x, 0.000, -0.145)),
            material=aluminium,
            name=f"fork_cheek_{i}",
        )
        fork.visual(
            Cylinder(radius=0.019, length=0.016),
            origin=Origin(xyz=(x, 0.000, -0.246), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminium,
            name=f"axle_boss_{i}",
        )
    fork.visual(
        Cylinder(radius=0.011, length=0.126),
        origin=Origin(xyz=(0.0, 0.000, -0.246), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_pin",
    )

    wheel = model.part("wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.118,
            0.056,
            inner_radius=0.083,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.08),
            tread=TireTread(style="ribbed", depth=0.004, count=20, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.005, radius=0.004),
        ),
        "black_ribbed_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.087,
            0.050,
            rim=WheelRim(inner_radius=0.060, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.031,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.038, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "machined_wheel_hub",
    )
    wheel.visual(tire_mesh, material=dark, name="tire")
    wheel.visual(wheel_mesh, material=aluminium, name="wheel_core")

    retract = model.articulation(
        "mount_to_strut",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=main,
        origin=Origin(xyz=(0.0, 0.004, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "mount_to_brace",
        ArticulationType.REVOLUTE,
        parent=mount,
        child=brace,
        origin=Origin(xyz=(0.130, 0.000, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=0.0, upper=0.90),
        mimic=Mimic(joint=retract.name, multiplier=0.55, offset=0.0),
    )
    model.articulation(
        "strut_to_slider",
        ArticulationType.PRISMATIC,
        parent=main,
        child=slider,
        origin=Origin(xyz=(0.0, 0.000, -0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=0.35, lower=0.0, upper=0.060),
    )
    model.articulation(
        "slider_to_fork",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=fork,
        origin=Origin(xyz=(0.0, 0.000, -0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "fork_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.000, -0.246)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount = object_model.get_part("mount_plate")
    main = object_model.get_part("main_strut")
    brace = object_model.get_part("folding_brace")
    slider = object_model.get_part("lower_slider")
    fork = object_model.get_part("fork")
    wheel = object_model.get_part("wheel")
    retract = object_model.get_articulation("mount_to_strut")
    shock = object_model.get_articulation("strut_to_slider")
    caster = object_model.get_articulation("slider_to_fork")
    spin = object_model.get_articulation("fork_to_wheel")

    ctx.warn(
        "Classification note: folder context says Astronomy, but the reference image depicts "
        "retractable landing gear; this asset follows the visible landing gear."
    )

    ctx.allow_overlap(
        mount,
        main,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The retraction hinge pin is intentionally captured inside the strut knuckle.",
    )
    ctx.expect_within(
        main,
        mount,
        axes="x",
        inner_elem="hinge_knuckle",
        outer_elem="hinge_pin",
        margin=0.001,
        name="strut knuckle is captured along hinge pin",
    )
    ctx.expect_overlap(
        main,
        mount,
        axes="yz",
        elem_a="hinge_knuckle",
        elem_b="hinge_pin",
        min_overlap=0.010,
        name="hinge pin passes through strut knuckle",
    )

    ctx.allow_overlap(
        main,
        slider,
        elem_a="outer_tube",
        elem_b="piston",
        reason="The chrome shock piston is modeled as retained inside a simplified outer sleeve.",
    )
    ctx.expect_within(
        slider,
        main,
        axes="xy",
        inner_elem="piston",
        outer_elem="outer_tube",
        margin=0.002,
        name="shock piston stays centered in outer tube",
    )
    ctx.expect_overlap(
        slider,
        main,
        axes="z",
        elem_a="piston",
        elem_b="outer_tube",
        min_overlap=0.050,
        name="shock piston has retained insertion",
    )
    ctx.allow_overlap(
        main,
        slider,
        elem_a="lower_collar",
        elem_b="piston",
        reason="The lower collar is a guide bushing around the sliding shock piston.",
    )
    ctx.expect_within(
        slider,
        main,
        axes="xy",
        inner_elem="piston",
        outer_elem="lower_collar",
        margin=0.002,
        name="piston is centered through lower guide collar",
    )

    ctx.allow_overlap(
        fork,
        wheel,
        elem_a="axle_pin",
        elem_b="wheel_core",
        reason="The wheel rotates around a captured axle through the hub bore.",
    )
    ctx.expect_overlap(
        fork,
        wheel,
        axes="x",
        elem_a="axle_pin",
        elem_b="wheel_core",
        min_overlap=0.045,
        name="axle spans through wheel hub",
    )
    ctx.expect_within(
        wheel,
        fork,
        axes="x",
        inner_elem="tire",
        margin=0.001,
        name="tire sits between fork cheeks",
    )
    ctx.allow_overlap(
        slider,
        fork,
        elem_a="lower_bushing",
        elem_b="fork_bridge",
        reason="The caster fork bridge is seated around the lower steering bushing.",
    )
    ctx.expect_overlap(
        slider,
        fork,
        axes="xy",
        elem_a="lower_bushing",
        elem_b="fork_bridge",
        min_overlap=0.015,
        name="caster fork bridge captures lower bushing",
    )

    ctx.allow_overlap(
        mount,
        brace,
        elem_a="brace_upper_pin",
        elem_b="upper_eye",
        reason="The side brace upper eye is pinned to the mount lug.",
    )
    ctx.expect_overlap(
        mount,
        brace,
        axes="y",
        elem_a="brace_upper_pin",
        elem_b="upper_eye",
        min_overlap=0.015,
        name="brace upper eye surrounds pin",
    )

    ctx.allow_overlap(
        main,
        brace,
        elem_a="lower_collar",
        elem_b="lower_eye",
        reason="The folding brace lower eye is shown seated on the lower strut collar pin point.",
    )
    ctx.expect_overlap(
        main,
        brace,
        axes="xz",
        elem_a="lower_collar",
        elem_b="lower_eye",
        min_overlap=0.010,
        name="brace lower eye meets strut collar",
    )

    ctx.check(
        "landing gear articulations present",
        caster.articulation_type == ArticulationType.REVOLUTE
        and spin.articulation_type == ArticulationType.CONTINUOUS
        and retract.articulation_type == ArticulationType.REVOLUTE,
        details=f"caster={caster.articulation_type}, spin={spin.articulation_type}, retract={retract.articulation_type}",
    )
    ctx.expect_origin_gap(
        mount,
        wheel,
        axis="z",
        min_gap=0.85,
        name="long strut places wheel well below mount",
    )

    rest_wheel = ctx.part_world_position(wheel)
    with ctx.pose({retract: 1.15}):
        folded_wheel = ctx.part_world_position(wheel)
    ctx.check(
        "retraction hinge folds wheel upward and aft",
        rest_wheel is not None
        and folded_wheel is not None
        and folded_wheel[2] > rest_wheel[2] + 0.35
        and folded_wheel[1] > rest_wheel[1] + 0.55,
        details=f"rest={rest_wheel}, folded={folded_wheel}",
    )

    rest_slider = ctx.part_world_position(slider)
    with ctx.pose({shock: 0.050}):
        compressed_slider = ctx.part_world_position(slider)
        ctx.expect_overlap(
            slider,
            main,
            axes="z",
            elem_a="piston",
            elem_b="outer_tube",
            min_overlap=0.070,
            name="compressed shock remains inserted",
        )
    ctx.check(
        "shock prismatic motion compresses upward",
        rest_slider is not None
        and compressed_slider is not None
        and compressed_slider[2] > rest_slider[2] + 0.045,
        details=f"rest={rest_slider}, compressed={compressed_slider}",
    )

    return ctx.report()


object_model = build_object_model()
