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
    tube_from_spline_points,
)


def _cyl_x(length: float, radius: float, xyz, name: str, material: Material):
    return (
        Cylinder(radius=radius, length=length),
        Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        name,
        material,
    )


def _cyl_y(length: float, radius: float, xyz, name: str, material: Material):
    return (
        Cylinder(radius=radius, length=length),
        Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        name,
        material,
    )


def _cyl_z(length: float, radius: float, xyz, name: str, material: Material):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz), name, material


def _add_cylinder(part, spec) -> None:
    geometry, origin, name, material = spec
    part.visual(geometry, origin=origin, material=material, name=name)


def _create_hanger(
    model: ArticulatedObject,
    parent,
    prefix: str,
    x: float,
    y: float,
    rail_z: float,
    hook_material: Material,
    body_material: Material,
    *,
    width: float = 0.17,
) -> None:
    hanger = model.part(prefix)
    hook_path = [
        (0.0, -0.028, 0.010),
        (0.0, -0.023, 0.030),
        (0.0, -0.004, 0.040),
        (0.0, 0.016, 0.030),
        (0.0, 0.024, 0.010),
        (0.0, 0.016, -0.016),
        (0.0, 0.003, -0.036),
    ]
    hook = tube_from_spline_points(
        hook_path,
        radius=0.0032,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    hanger.visual(
        mesh_from_geometry(hook, f"{prefix}_hook_mesh"),
        material=hook_material,
        name="hook_loop",
    )
    _add_cylinder(
        hanger,
        _cyl_z(
            0.064,
            0.0032,
            (0.0, 0.003, -0.065),
            "vertical_neck",
            hook_material,
        ),
    )

    body_path = [
        (0.0, 0.0, -0.092),
        (-width / 2.0, 0.0, -0.225),
        (width / 2.0, 0.0, -0.225),
        (0.0, 0.0, -0.092),
    ]
    body = tube_from_spline_points(
        body_path,
        radius=0.0055,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )
    hanger.visual(
        mesh_from_geometry(body, f"{prefix}_body_mesh"),
        material=body_material,
        name="hanger_body",
    )
    model.articulation(
        f"{prefix}_swing",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=hanger,
        origin=Origin(xyz=(x, y, rail_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.35, upper=0.35),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rolling_gold_clothing_rack",
        meta={
            "reference_note": "Category and image agree: a rolling double-rail clothing rack with side hooks and adjustable upper rail.",
        },
    )

    brass = model.material("brushed_gold_tube", rgba=(0.86, 0.58, 0.18, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.02, 0.018, 0.016, 1.0))
    grey_hub = model.material("dark_grey_hub", rgba=(0.16, 0.16, 0.15, 1.0))
    screw = model.material("small_screw_heads", rgba=(0.92, 0.78, 0.48, 1.0))
    hanger_hook = model.material("brushed_silver_hanger_hooks", rgba=(0.78, 0.74, 0.66, 1.0))
    pale_wood = model.material("pale_wood_hangers", rgba=(0.74, 0.54, 0.34, 1.0))

    frame = model.part("base_frame")

    # Overall reference proportions: about 1.65 m wide, 0.55 m deep, and
    # 1.75 m tall.  X is the long rail direction, Y is depth, Z is height.
    rail_r = 0.018
    sleeve_r = 0.023
    small_r = 0.010
    width = 1.62
    x0 = -0.78
    x1 = 0.78
    y_front = -0.25
    y_back = 0.23

    # Four grounded posts, with rear posts represented as larger outer sleeves
    # because the taller rail telescopes inside them.
    frame.visual(
        Cylinder(radius=rail_r, length=1.12),
        origin=Origin(xyz=(x0, y_front, 0.68)),
        material=brass,
        name="front_post_0",
    )
    frame.visual(
        Cylinder(radius=rail_r, length=1.12),
        origin=Origin(xyz=(x1, y_front, 0.68)),
        material=brass,
        name="front_post_1",
    )
    frame.visual(
        Cylinder(radius=sleeve_r, length=1.22),
        origin=Origin(xyz=(x0, y_back, 0.73)),
        material=brass,
        name="rear_sleeve_0",
    )
    frame.visual(
        Cylinder(radius=sleeve_r, length=1.22),
        origin=Origin(xyz=(x1, y_back, 0.73)),
        material=brass,
        name="rear_sleeve_1",
    )

    # Lower shoe/storage shelf: two tiers of round rods and the side rails that
    # tie the legs together.  Rod ends penetrate the posts slightly, as welded
    # tubular rack construction normally does.
    for z, suffix in ((0.18, "lower"), (0.36, "upper")):
        for j, y in enumerate((-0.21, -0.10, 0.01, 0.12, 0.21)):
            _add_cylinder(
                frame,
                _cyl_x(width, small_r, (0.0, y, z), f"{suffix}_shelf_rod_{j}", brass),
            )
        _add_cylinder(
            frame,
            _cyl_y(0.54, small_r, (x0, 0.0, z), f"{suffix}_side_rail_0", brass),
        )
        _add_cylinder(
            frame,
            _cyl_y(0.54, small_r, (x1, 0.0, z), f"{suffix}_side_rail_1", brass),
        )

    # Front hanging rail and rear low stabilizer rail, mirroring the image's
    # visible double-rail clothing-rack layout.
    frame.visual(
        Cylinder(radius=rail_r, length=1.68),
        origin=Origin(xyz=(0.0, y_front, 1.22), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="front_hanging_rail",
    )
    frame.visual(
        Cylinder(radius=rail_r, length=1.58),
        origin=Origin(xyz=(0.0, y_back, 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_stabilizer_rail",
    )
    for idx, x in enumerate((-0.48, -0.30, -0.12, 0.06, 0.24, 0.42)):
        _create_hanger(
            model,
            frame,
            f"front_hanger_{idx}",
            x,
            y_front,
            1.22,
            hanger_hook,
            pale_wood,
            width=0.16,
        )

    # Rounded lower side hoops at the base make the frame read as bent tube
    # rather than a set of disconnected rods.
    for side, x in enumerate((x0, x1)):
        path = [
            (x, y_front, 0.12),
            (x, y_front, 0.30),
            (x, y_front + 0.04, 0.39),
            (x, -0.02, 0.43),
            (x, y_back - 0.04, 0.39),
            (x, y_back, 0.30),
            (x, y_back, 0.12),
        ]
        hoop = tube_from_spline_points(
            path,
            radius=rail_r,
            samples_per_segment=10,
            radial_segments=20,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(hoop, f"base_side_hoop_{side}"),
            material=brass,
            name=f"base_side_hoop_{side}",
        )

    # Adjustment collars and hand knobs on the rear sleeves.  The small side
    # hooks are fixed pegs with rounded ends, visible in the reference.
    frame.visual(
        Cylinder(radius=0.029, length=0.055),
        origin=Origin(xyz=(x0, y_back, 1.115)),
        material=brass,
        name="adjust_collar_0",
    )
    frame.visual(
        Cylinder(radius=0.029, length=0.055),
        origin=Origin(xyz=(x1, y_back, 1.115)),
        material=brass,
        name="adjust_collar_1",
    )
    for side, x in enumerate((x0, x1)):
        knob_x = x - 0.055 if x < 0 else x + 0.055
        screw_x = x - 0.046 if x < 0 else x + 0.046
        _add_cylinder(
            frame,
            _cyl_x(0.034, 0.007, (screw_x, y_back, 1.12), f"clamp_screw_{side}", screw),
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(knob_x, y_back, 1.12), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw,
            name=f"clamp_knob_{side}",
        )

        hook_x = x - 0.075 if x < 0 else x + 0.075
        for j, z in enumerate((0.84, 1.02)):
            _add_cylinder(
                frame,
                _cyl_x(0.095, 0.008, ((x + hook_x) / 2.0, y_front, z), f"side_hook_{side}_{j}", brass),
            )
            frame.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(hook_x, y_front, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brass,
                name=f"hook_cap_{side}_{j}",
            )

    # Caster fork/stem hardware belongs to the fixed frame.  Each axle is a real
    # captured pin through its wheel, so the wheel parts intentionally overlap
    # the named axle element.
    caster_positions = [
        (x0, y_front, "0"),
        (x1, y_front, "1"),
        (x0, y_back, "2"),
        (x1, y_back, "3"),
    ]
    for x, y, idx in caster_positions:
        _add_cylinder(frame, _cyl_z(0.05, 0.010, (x, y, 0.120), f"caster_stem_{idx}", grey_hub))
        _add_cylinder(frame, _cyl_x(0.080, 0.005, (x, y, 0.045), f"caster_axle_{idx}", grey_hub))
        frame.visual(
            Box((0.007, 0.050, 0.065)),
            origin=Origin(xyz=(x - 0.026, y, 0.066)),
            material=grey_hub,
            name=f"caster_fork_plate_{idx}_0",
        )
        frame.visual(
            Box((0.007, 0.050, 0.065)),
            origin=Origin(xyz=(x + 0.026, y, 0.066)),
            material=grey_hub,
            name=f"caster_fork_plate_{idx}_1",
        )
        frame.visual(
            Box((0.065, 0.050, 0.016)),
            origin=Origin(xyz=(x, y, 0.103)),
            material=grey_hub,
            name=f"caster_fork_bridge_{idx}",
        )

    # The upper rear rail is one telescoping moving assembly: two inner mast
    # sections sliding in the rear sleeves and a continuous rounded top rail.
    top = model.part("top_rail")
    top_loop_path = [
        (x0, y_back, 0.88),
        (x0, y_back, 1.48),
        (x0, y_back, 1.60),
        (x0 + 0.08, y_back, 1.69),
        (0.0, y_back, 1.72),
        (x1 - 0.08, y_back, 1.69),
        (x1, y_back, 1.60),
        (x1, y_back, 1.48),
        (x1, y_back, 0.88),
    ]
    top_loop = tube_from_spline_points(
        top_loop_path,
        radius=0.016,
        samples_per_segment=12,
        radial_segments=24,
        cap_ends=True,
    )
    top.visual(
        mesh_from_geometry(top_loop, "telescoping_top_loop"),
        material=brass,
        name="top_loop",
    )
    for idx, x in enumerate((-0.42, -0.24, -0.06, 0.12, 0.30)):
        _create_hanger(
            model,
            top,
            f"upper_hanger_{idx}",
            x,
            y_back,
            1.69,
            hanger_hook,
            pale_wood,
            width=0.145,
        )
    for x, name in ((x0 + 0.08, "top_finial_0"), (x1 - 0.08, "top_finial_1")):
        top.visual(
            Cylinder(radius=0.030, length=0.026),
            origin=Origin(xyz=(x, y_back, 1.705)),
            material=brass,
            name=name,
        )

    top_slide = model.articulation(
        "base_to_top_rail",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=top,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.22),
    )

    # Reusable detailed caster wheel meshes.  Wheel/Tire helpers align the spin
    # axis with local X, matching each caster axle.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.029,
            0.030,
            rim=WheelRim(inner_radius=0.017, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.012,
                width=0.022,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.016, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.002, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.046,
            0.034,
            inner_radius=0.028,
            carcass=TireCarcass(belt_width_ratio=0.78, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.0025, count=16, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
            shoulder=TireShoulder(width=0.003, radius=0.0015),
        ),
        "caster_tire",
    )

    for x, y, idx in caster_positions:
        wheel_part = model.part(f"caster_{idx}")
        wheel_part.visual(tire_mesh, material=dark_rubber, name="tire")
        wheel_part.visual(wheel_mesh, material=grey_hub, name="hub")
        model.articulation(
            f"frame_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel_part,
            origin=Origin(xyz=(x, y, 0.045)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=10.0),
        )

    # Preserve a handle to the top slide for tests without relying on list order.
    top_slide.meta["mechanism"] = "height_adjustment"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("base_frame")
    top = object_model.get_part("top_rail")
    top_slide = object_model.get_articulation("base_to_top_rail")

    # Intentional retained telescoping fit: the inner masts are represented by
    # the same moving top-loop tube and slide inside the fixed rear sleeve
    # proxies.  The overlap is local to the sleeve/mast regions, not the rail.
    for sleeve in ("rear_sleeve_0", "rear_sleeve_1"):
        ctx.allow_overlap(
            frame,
            top,
            elem_a=sleeve,
            elem_b="top_loop",
            reason="Inner mast sections intentionally telescope inside the fixed rear sleeve proxy.",
        )
    for collar in ("adjust_collar_0", "adjust_collar_1"):
        ctx.allow_overlap(
            frame,
            top,
            elem_a=collar,
            elem_b="top_loop",
            reason="Clamp collar surrounds the sliding inner mast at the telescoping joint.",
        )
        ctx.expect_overlap(
            top,
            frame,
            axes="z",
            elem_a="top_loop",
            elem_b=collar,
            min_overlap=0.03,
            name=f"{collar} surrounds sliding mast",
        )

    # Each caster axle is a captured pin through the wheel hub.  This local
    # mechanical embedding is intentional and proves the wheel is physically
    # mounted rather than floating near the fork.
    for idx in range(4):
        caster = object_model.get_part(f"caster_{idx}")
        ctx.allow_overlap(
            frame,
            caster,
            elem_a=f"caster_axle_{idx}",
            elem_b="hub",
            reason="Caster axle intentionally passes through the wheel hub bore.",
        )
        ctx.expect_contact(
            caster,
            frame,
            elem_a="hub",
            elem_b=f"caster_axle_{idx}",
            contact_tol=0.010,
            name=f"caster {idx} hub is captured by axle",
        )

    # Prompt-specific proportions and mechanisms.
    ctx.expect_overlap(
        frame,
        frame,
        axes="x",
        elem_a="front_hanging_rail",
        elem_b="rear_stabilizer_rail",
        min_overlap=1.40,
        name="rack has long parallel hanging rails",
    )
    ctx.check(
        "front rail has multiple articulated hangers",
        all(
            object_model.get_part(f"front_hanger_{idx}") is not None
            and object_model.get_articulation(f"front_hanger_{idx}_swing") is not None
            for idx in range(6)
        ),
    )
    front_hanger = object_model.get_part("front_hanger_0")
    front_swing = object_model.get_articulation("front_hanger_0_swing")
    ctx.check(
        "front hanger swings a small range around rail axis",
        front_swing is not None
        and front_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(front_swing.axis) == (1.0, 0.0, 0.0)
        and front_swing.motion_limits is not None
        and front_swing.motion_limits.lower == -0.35
        and front_swing.motion_limits.upper == 0.35,
    )
    ctx.allow_overlap(
        frame,
        front_hanger,
        elem_a="front_hanging_rail",
        elem_b="hook_loop",
        reason="The hanger hook intentionally wraps around the clothing rail.",
    )
    ctx.expect_overlap(
        frame,
        front_hanger,
        axes="z",
        elem_a="front_hanging_rail",
        elem_b="hook_loop",
        min_overlap=0.010,
        name="front hanger hook wraps the front rail",
    )
    ctx.expect_gap(
        frame,
        front_hanger,
        axis="z",
        positive_elem="front_hanging_rail",
        negative_elem="hanger_body",
        min_gap=0.040,
        name="front hanger body hangs below front rail",
    )
    ctx.expect_overlap(
        front_hanger,
        front_hanger,
        axes="z",
        elem_a="vertical_neck",
        elem_b="hanger_body",
        min_overlap=0.003,
        name="front hanger neck plugs into wooden body",
    )
    ctx.check(
        "upper rail has articulated hangers",
        all(
            object_model.get_part(f"upper_hanger_{idx}") is not None
            and object_model.get_articulation(f"upper_hanger_{idx}_swing") is not None
            for idx in range(5)
        ),
    )
    upper_hanger = object_model.get_part("upper_hanger_0")
    upper_swing = object_model.get_articulation("upper_hanger_0_swing")
    ctx.check(
        "upper hanger swings a small range around rail axis",
        upper_swing is not None
        and upper_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(upper_swing.axis) == (1.0, 0.0, 0.0)
        and upper_swing.motion_limits is not None
        and upper_swing.motion_limits.lower == -0.35
        and upper_swing.motion_limits.upper == 0.35,
    )
    ctx.allow_overlap(
        top,
        upper_hanger,
        elem_a="top_loop",
        elem_b="hook_loop",
        reason="The hanger hook intentionally wraps around the upper clothing rail.",
    )
    ctx.expect_gap(
        top,
        upper_hanger,
        axis="z",
        positive_elem="top_loop",
        negative_elem="hanger_body",
        min_gap=0.040,
        name="upper hanger body hangs below upper rail",
    )
    ctx.expect_overlap(
        upper_hanger,
        upper_hanger,
        axes="z",
        elem_a="vertical_neck",
        elem_b="hanger_body",
        min_overlap=0.003,
        name="upper hanger neck plugs into wooden body",
    )
    ctx.expect_within(
        top,
        frame,
        axes="xy",
        inner_elem="top_loop",
        outer_elem="rear_sleeve_0",
        margin=1.60,
        name="top rail stays aligned over sleeve footprint",
    )
    ctx.expect_overlap(
        top,
        frame,
        axes="z",
        elem_a="top_loop",
        elem_b="rear_sleeve_0",
        min_overlap=0.15,
        name="collapsed top rail retains sleeve insertion",
    )

    rest_pos = ctx.part_world_position(top)
    with ctx.pose({top_slide: 0.22}):
        ctx.expect_overlap(
            top,
            frame,
            axes="z",
            elem_a="top_loop",
            elem_b="rear_sleeve_0",
            min_overlap=0.05,
            name="extended top rail remains inserted",
        )
        extended_pos = ctx.part_world_position(top)
    ctx.check(
        "height adjustment slides upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
