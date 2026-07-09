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
    mesh_from_geometry,
    tube_from_spline_points,
)

BLACK_METAL = "slightly_gloss_black_powder_coated_steel"
RED_CAP = "red_plastic_ball_caps"
RUBBER = "matte_black_rubber"
DARK_HUB = "dark_grey_hub_hardware"


def _hollow_z_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Open-ended vertical annular tube, authored in meters."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _hollow_y_tube(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Open-ended annular wheel/tire blank whose spin axis is local Y."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width)
        .translate((0.0, -width / 2.0, 0.0))
    )


def _add_cylinder_x(part, name: str, length: float, radius: float, xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, name: str, length: float, radius: float, xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_z(part, name: str, length: float, radius: float, xyz, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_wheel_visuals(part, tire_mat: Material, hub_mat: Material, tire_name: str) -> None:
    # Caster wheel axis is local/world Y. Cylinders are rotated so their local Z
    # length becomes the visible wheel width.
    part.visual(
        mesh_from_cadquery(
            _hollow_y_tube(0.043, 0.025, 0.032),
            tire_name,
            tolerance=0.0008,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=tire_mat,
        name="rubber_tire",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub_disc",
    )


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
        (0.0, -0.018, 0.018),
        (0.0, -0.010, 0.040),
        (0.0, 0.016, 0.036),
        (0.0, 0.028, 0.014),
        (0.0, 0.018, -0.012),
        (0.0, 0.003, -0.032),
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
    _add_cylinder_z(
        hanger,
        "vertical_neck",
        0.064,
        0.0032,
        (0.0, 0.002, -0.054),
        hook_material,
    )

    body_path = [
        (0.0, 0.0, -0.074),
        (0.0, -width / 2.0, -0.225),
        (0.0, width / 2.0, -0.225),
        (0.0, 0.0, -0.074),
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
        name="retail_clothing_rack",
        meta={
            "category": "Retail_Shop Fixtures / Clothing rack",
            "reference_note": "The visible core object is a wheeled adjustable clothing rack; garments and props from the reference are intentionally omitted.",
        },
    )

    black = model.material(BLACK_METAL, rgba=(0.005, 0.006, 0.006, 1.0))
    red = model.material(RED_CAP, rgba=(0.82, 0.06, 0.035, 1.0))
    rubber = model.material(RUBBER, rgba=(0.006, 0.006, 0.006, 1.0))
    hub = model.material(DARK_HUB, rgba=(0.09, 0.09, 0.085, 1.0))
    hanger_hook = model.material("brushed_silver_hanger_hooks", rgba=(0.78, 0.74, 0.66, 1.0))
    pale_wood = model.material("pale_wood_hangers", rgba=(0.74, 0.54, 0.34, 1.0))

    base_frame = model.part("base_frame")

    # Overall dimensions are a believable retail rack: about 1.65 m wide,
    # 0.52 m deep, and 1.7--1.85 m tall depending on the telescoping height.
    tube_r = 0.022
    base_z = 0.105
    x_post = 0.72
    y_front = -0.245
    y_rear = 0.245

    # Rectangular tubular base with rounded ball-like welded corner elbows.
    _add_cylinder_x(base_frame, "front_base_tube", 1.48, tube_r, (0.0, y_front, base_z), black)
    _add_cylinder_x(base_frame, "rear_base_tube", 1.48, tube_r, (0.0, y_rear, base_z), black)
    _add_cylinder_y(base_frame, "base_tube_0", 0.49, tube_r, (-x_post, 0.0, base_z), black)
    _add_cylinder_y(base_frame, "base_tube_1", 0.49, tube_r, (x_post, 0.0, base_z), black)
    for i, (x, y) in enumerate(
        ((-x_post, y_front), (-x_post, y_rear), (x_post, y_front), (x_post, y_rear))
    ):
        base_frame.visual(
            Sphere(radius=tube_r),
            origin=Origin(xyz=(x, y, base_z)),
            material=black,
            name=f"rounded_corner_{i}",
        )

    # Open steel sleeves receive the smaller upper posts without a false solid
    # overlap. They are mesh-backed annular tubes with visible wall thickness.
    lower_sleeve_mesh = mesh_from_cadquery(
        _hollow_z_tube(0.0235, 0.0175, 0.94),
        "lower_telescoping_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    clamp_collar_mesh = mesh_from_cadquery(
        _hollow_z_tube(0.027, 0.018, 0.036),
        "split_clamp_collar",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    for i, x in enumerate((-x_post, x_post)):
        base_frame.visual(
            lower_sleeve_mesh,
            origin=Origin(xyz=(x, 0.0, 0.10)),
            material=black,
            name=f"lower_sleeve_{i}",
        )
        # Slim clamp collars and spring buttons make the height adjustment read
        # as a real telescoping rack.
        base_frame.visual(
            clamp_collar_mesh,
            origin=Origin(xyz=(x, 0.0, 1.004)),
            material=black,
            name=f"clamp_collar_{i}",
        )
        _add_cylinder_y(base_frame, f"spring_button_{i}", 0.014, 0.005, (x, -0.027, 0.72), hub)

    # Low rear brace echoing the reference rack's bottom support tube.
    _add_cylinder_x(base_frame, "low_rear_brace", 1.44, 0.017, (0.0, 0.0, 0.235), black)

    # Four caster forks are fixed to the base; each wheel is a separate rotating part.
    caster_positions = (
        (-0.64, y_front, 0.043),
        (-0.64, y_rear, 0.043),
        (0.64, y_front, 0.043),
        (0.64, y_rear, 0.043),
    )
    for i, (x, y, z) in enumerate(caster_positions):
        _add_cylinder_z(base_frame, f"caster_stem_{i}", 0.060, 0.007, (x, y, 0.122), black)
        base_frame.visual(
            Box((0.030, 0.064, 0.008)),
            origin=Origin(xyz=(x, y, 0.094)),
            material=black,
            name=f"caster_bridge_{i}",
        )
        _add_cylinder_y(base_frame, f"caster_cross_pin_{i}", 0.070, 0.006, (x, y, z), black)
        for side, yy in enumerate((y - 0.026, y + 0.026)):
            base_frame.visual(
                Box((0.030, 0.006, 0.078)),
                origin=Origin(xyz=(x, yy, 0.059)),
                material=black,
                name=f"caster_fork_{i}_{side}",
            )

    upper_frame = model.part("upper_frame")
    # The arch crown sits slightly above the original straight-rail height so
    # the tube reads as a continuous bow sweeping up each side and across.
    crown_z = 0.70
    arch_leg_bottom_z = -0.24  # deep insertion into lower sleeves for telescoping travel

    # Sliding bushings at the bottom of each arch leg provide the telescoping
    # guide fit inside the lower sleeves.
    for i, x in enumerate((-x_post, x_post)):
        _add_cylinder_z(
            upper_frame,
            f"sliding_bushing_{i}",
            0.080,
            0.01755,
            (x, 0.0, arch_leg_bottom_z),
            hub,
        )

    # Single continuous arched inverted-U tube built from a Catmull-Rom spline.
    # The path rises vertically from each sleeve, curves with generous radii at
    # the shoulders, and forms a level horizontal crown for the hangers.
    arch_path = [
        (-x_post, 0.0, arch_leg_bottom_z),
        (-x_post, 0.0, 0.05),
        (-x_post, 0.0, 0.25),
        (-x_post, 0.0, 0.42),
        (-x_post + 0.05, 0.0, 0.52),
        (-x_post + 0.15, 0.0, 0.62),
        (-x_post + 0.30, 0.0, crown_z - 0.02),
        (-0.30, 0.0, crown_z),
        (-0.10, 0.0, crown_z),
        (0.10, 0.0, crown_z),
        (0.30, 0.0, crown_z),
        (x_post - 0.30, 0.0, crown_z - 0.02),
        (x_post - 0.15, 0.0, 0.62),
        (x_post - 0.05, 0.0, 0.52),
        (x_post, 0.0, 0.42),
        (x_post, 0.0, 0.25),
        (x_post, 0.0, 0.05),
        (x_post, 0.0, arch_leg_bottom_z),
    ]
    arch_mesh = mesh_from_geometry(
        tube_from_spline_points(
            arch_path,
            radius=0.021,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "arch_rail_mesh",
    )
    upper_frame.visual(
        arch_mesh,
        material=black,
        name="arch_rail",
    )

    # Hangers sit on the level crown section, well inside the arch curve radius.
    for idx, x in enumerate((-0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36)):
        _create_hanger(
            model,
            upper_frame,
            f"top_hanger_{idx}",
            x,
            -0.016,
            crown_z,
            hanger_hook,
            pale_wood,
            width=0.16,
        )

    # Right-side accessory hooks with red ball caps, mounted on the arch leg area.
    hook_base_z = 0.50
    _add_cylinder_z(upper_frame, "upright_side_hook", 0.22, 0.017, (x_post, 0.0, hook_base_z + 0.11), black)
    upper_frame.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(x_post, 0.0, hook_base_z + 0.235)),
        material=red,
        name="top_red_ball",
    )
    _add_cylinder_x(upper_frame, "side_hook", 0.30, 0.016, (x_post + 0.15, 0.0, hook_base_z), black)
    upper_frame.visual(
        Sphere(radius=0.031),
        origin=Origin(xyz=(x_post + 0.315, 0.0, hook_base_z)),
        material=red,
        name="side_red_ball",
    )

    # Small underside stops on the arch crown, visible as dark nubs pressed
    # into the tube underside for a connected mount. Spaced between hangers to
    # avoid contact.
    for i, x in enumerate((-0.18, 0.18)):
        upper_frame.visual(
            Box((0.036, 0.018, 0.010)),
            origin=Origin(xyz=(x, 0.0, crown_z - 0.023)),
            material=black,
            name=f"hanger_stop_{i}",
        )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=upper_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.14),
    )

    for i, (x, y, z) in enumerate(caster_positions):
        wheel = model.part(f"wheel_{i}")
        _add_wheel_visuals(wheel, rubber, hub, f"caster_tire_{i}")
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    upper = object_model.get_part("upper_frame")
    height = object_model.get_articulation("height_slide")

    ctx.check(
        "rack has four caster wheels",
        all(object_model.get_part(f"wheel_{i}") is not None for i in range(4)),
    )
    ctx.check(
        "rack has wheel spin joints",
        all(object_model.get_articulation(f"wheel_spin_{i}") is not None for i in range(4)),
    )
    for i in range(4):
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            base,
            wheel,
            elem_a=f"caster_cross_pin_{i}",
            elem_b="hub_disc",
            reason="The fixed caster axle is intentionally captured through the rotating wheel hub.",
        )
        ctx.expect_overlap(
            base,
            wheel,
            axes="y",
            elem_a=f"caster_cross_pin_{i}",
            elem_b="hub_disc",
            min_overlap=0.030,
            name=f"caster axle {i} passes through wheel hub",
        )

    # The arch rail legs intentionally pass through the lower sleeves and clamp
    # collars as part of the telescoping mechanism.
    for i in range(2):
        ctx.allow_overlap(
            base,
            upper,
            elem_a=f"lower_sleeve_{i}",
            elem_b="arch_rail",
            reason="The arch rail leg intentionally slides through the lower sleeve as part of the telescoping height adjustment.",
        )
        ctx.allow_overlap(
            base,
            upper,
            elem_a=f"clamp_collar_{i}",
            elem_b="arch_rail",
            reason="The clamp collar encircles the arch rail leg at the sleeve top for a realistic telescoping fit.",
        )
        ctx.allow_overlap(
            base,
            upper,
            elem_a=f"lower_sleeve_{i}",
            elem_b=f"sliding_bushing_{i}",
            reason="The hidden plastic guide bushing is intentionally captured in the telescoping sleeve for a snug sliding fit.",
        )
        ctx.expect_within(
            upper,
            base,
            axes="xy",
            inner_elem=f"sliding_bushing_{i}",
            outer_elem=f"lower_sleeve_{i}",
            margin=0.002,
            name=f"sliding bushing {i} is captured inside sleeve",
        )
        ctx.expect_overlap(
            upper,
            base,
            axes="z",
            elem_a=f"sliding_bushing_{i}",
            elem_b=f"lower_sleeve_{i}",
            min_overlap=0.070,
            name=f"sliding bushing {i} remains inserted at rest",
        )

    rest_aabb = ctx.part_world_aabb(upper)
    with ctx.pose({height: 0.14}):
        raised_aabb = ctx.part_world_aabb(upper)
        for i in range(2):
            ctx.expect_within(
                upper,
                base,
                axes="xy",
                inner_elem=f"sliding_bushing_{i}",
                outer_elem=f"lower_sleeve_{i}",
                margin=0.002,
                name=f"raised sliding bushing {i} remains centered",
            )
            ctx.expect_overlap(
                upper,
                base,
                axes="z",
                elem_a=f"sliding_bushing_{i}",
                elem_b=f"lower_sleeve_{i}",
                min_overlap=0.025,
                name=f"raised sliding bushing {i} retains insertion",
            )

    ctx.check(
        "height slide raises arch crown",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    # The arch rail is a continuous inverted-U tube; prove it spans the base
    # width and its crown is elevated above the base frame.
    ctx.check(
        "arch_rail visual exists on upper_frame",
        upper.get_visual("arch_rail") is not None,
        details="upper_frame should have a single arch_rail mesh visual",
    )
    ctx.expect_overlap(
        upper,
        base,
        axes="x",
        elem_a="arch_rail",
        elem_b="front_base_tube",
        min_overlap=1.30,
        name="arch rail spans most of wheeled base width",
    )
    # The arch crown (max Z) is well above the base; the arch legs extend down
    # into the sleeves so we use the AABB top to check crown height.
    ctx.check(
        "arch crown top is high above base",
        rest_aabb is not None and rest_aabb[1][2] > 1.60,
        details=f"rest_aabb_max_z={rest_aabb[1][2] if rest_aabb else None}",
    )

    ctx.check(
        "arch crown has multiple articulated hangers",
        all(
            object_model.get_part(f"top_hanger_{idx}") is not None
            and object_model.get_articulation(f"top_hanger_{idx}_swing") is not None
            for idx in range(7)
        ),
    )
    top_hanger = object_model.get_part("top_hanger_0")
    top_swing = object_model.get_articulation("top_hanger_0_swing")
    ctx.check(
        "top hanger swings a small range around rail axis",
        top_swing is not None
        and top_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(top_swing.axis) == (1.0, 0.0, 0.0)
        and top_swing.motion_limits is not None
        and top_swing.motion_limits.lower == -0.35
        and top_swing.motion_limits.upper == 0.35,
    )

    # Each hanger hook intentionally wraps around the curved arch crown tube,
    # and the vertical neck passes through the tube bore to connect hook to body.
    for idx in range(7):
        hanger_part = object_model.get_part(f"top_hanger_{idx}")
        ctx.allow_overlap(
            upper,
            hanger_part,
            elem_a="arch_rail",
            elem_b="hook_loop",
            reason="The hanger hook intentionally wraps around the curved arch crown.",
        )
        ctx.allow_overlap(
            upper,
            hanger_part,
            elem_a="arch_rail",
            elem_b="vertical_neck",
            reason="The hanger vertical neck passes through the arch crown tube to connect hook to body.",
        )
    ctx.expect_overlap(
        upper,
        top_hanger,
        axes="z",
        elem_a="arch_rail",
        elem_b="hook_loop",
        min_overlap=0.010,
        name="top hanger hook wraps the arch crown",
    )
    # The hanger body hangs well below the arch crown horizontal span.
    # Check that the hanger_body element's top is below the crown level.
    crown_z_world = 1.04 + 0.70  # upper_frame origin + crown_z offset
    hanger_body_aabb = ctx.part_element_world_aabb(top_hanger, elem="hanger_body")
    ctx.check(
        "top hanger body hangs below arch crown",
        hanger_body_aabb is not None and hanger_body_aabb[1][2] < crown_z_world - 0.05,
        details=f"crown_z_world={crown_z_world}, hanger_body_max_z={hanger_body_aabb[1][2] if hanger_body_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
