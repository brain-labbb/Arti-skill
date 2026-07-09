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


def _hollow_x_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Open-ended annular tube along local X, centered on origin."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
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
    rail_z = 0.66
    # Upper posts include hidden insertion length below the joint frame; at max
    # height they remain partially inside the lower sleeves.
    for i, x in enumerate((-x_post, x_post)):
        _add_cylinder_z(upper_frame, f"upper_post_{i}", 0.84, 0.014, (x, 0.0, 0.24), black)
        _add_cylinder_z(
            upper_frame,
            f"sliding_bushing_{i}",
            0.080,
            0.01755,
            (x, 0.0, -0.10),
            hub,
        )
        upper_frame.visual(
            Sphere(radius=0.025),
            origin=Origin(xyz=(x, 0.0, rail_z)),
            material=black,
            name=f"rail_tee_{i}",
        )

    # --- Telescoping horizontal rail ---
    # Outer sleeve: hollow annular tube spanning between the uprights (x=-0.72 to x=+0.72).
    # Length 1.44 m, centered at x=0. The rail_tee spheres at x=±0.72 cap both ends.
    rail_outer_mesh = mesh_from_cadquery(
        _hollow_x_tube(0.021, 0.017, 1.44),
        "rail_outer_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    upper_frame.visual(
        rail_outer_mesh,
        origin=Origin(xyz=(0.0, 0.0, rail_z)),
        material=black,
        name="rail_outer",
    )
    # Right-end bushing ring guides the inner rail and acts as a visible stop.
    rail_bushing_mesh = mesh_from_cadquery(
        _hollow_x_tube(0.0225, 0.0155, 0.030),
        "rail_right_bushing",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    upper_frame.visual(
        rail_bushing_mesh,
        origin=Origin(xyz=(0.705, 0.0, rail_z)),
        material=hub,
        name="rail_outer_bushing",
    )
    # Left-end cap sphere seats into the rail tee and closes the outer sleeve.
    upper_frame.visual(
        Sphere(radius=0.023),
        origin=Origin(xyz=(-0.72, 0.0, rail_z)),
        material=black,
        name="left_rail_cap",
    )

    # Hangers on the outer (fixed) rail section — only on the left portion
    # where the inner tube does not pass through. The inner tube at rest
    # extends from x=-0.08 to x=+0.72, so outer hangers stay at x < -0.10.
    for idx, x in enumerate((-0.60, -0.45, -0.30, -0.15)):
        _create_hanger(
            model,
            upper_frame,
            f"top_hanger_{idx}",
            x,
            -0.016,
            rail_z,
            hanger_hook,
            pale_wood,
            width=0.16,
        )

    # --- Inner sliding rail ---
    # Inner tube: solid cylinder that slides out along +X to extend the hanging span.
    # Length 0.80 m. At rest (q=0), centered at world x=0.32, spanning x=-0.08 to x=+0.72.
    # At max extension (q=0.25), centered at world x=0.57, spanning x=0.17 to x=0.97.
    rail_inner = model.part("rail_inner")
    # Inner tube: length 0.72 m so the right end stays at x=0.68 in world,
    # clearing the rail_tee_1 sphere at x=0.72 (radius 0.025).
    _add_cylinder_x(rail_inner, "inner_tube", 0.72, 0.015, (0.0, 0.0, 0.0), black)
    # Right end cap prevents the inner tube from fully withdrawing from the outer sleeve.
    rail_inner.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        material=black,
        name="inner_rail_cap",
    )
    # Left end stop ring (inside the outer sleeve) prevents full withdrawal.
    rail_inner_stop_mesh = mesh_from_cadquery(
        _hollow_x_tube(0.0165, 0.010, 0.018),
        "inner_rail_stop",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    rail_inner.visual(
        rail_inner_stop_mesh,
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        material=hub,
        name="inner_rail_stop",
    )

    # Hangers on the inner (sliding) rail — these move with rail_inner.
    # Local x positions place them on the right portion that extends past the outer sleeve.
    for idx, x_local in enumerate((0.08, 0.20, 0.28)):
        _create_hanger(
            model,
            rail_inner,
            f"inner_hanger_{idx}",
            x_local,
            -0.016,
            0.0,  # rail is at z=0 in rail_inner local frame
            hanger_hook,
            pale_wood,
            width=0.16,
        )

    model.articulation(
        "rail_extend",
        ArticulationType.PRISMATIC,
        parent=upper_frame,
        child=rail_inner,
        origin=Origin(xyz=(0.32, 0.0, rail_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.15, lower=0.0, upper=0.25),
    )

    # Right-side accessory hooks with red ball caps, matching the reference.
    _add_cylinder_z(upper_frame, "upright_side_hook", 0.22, 0.017, (x_post, 0.0, rail_z + 0.11), black)
    upper_frame.visual(
        Sphere(radius=0.032),
        origin=Origin(xyz=(x_post, 0.0, rail_z + 0.235)),
        material=red,
        name="top_red_ball",
    )
    _add_cylinder_x(upper_frame, "side_hook", 0.30, 0.016, (x_post + 0.15, 0.0, rail_z), black)
    upper_frame.visual(
        Sphere(radius=0.031),
        origin=Origin(xyz=(x_post + 0.315, 0.0, rail_z)),
        material=red,
        name="side_red_ball",
    )

    # Small underside stops on the hanging rail, visible as dark nubs in the reference.
    # Positioned to contact the outer sleeve underside and avoid all hanger hook locations.
    # Outer hangers at x=(-0.60, -0.45, -0.30, -0.15); inner hangers at rest at x=(0.40, 0.52, 0.60).
    for i, x in enumerate((-0.37, 0.05, 0.65)):
        upper_frame.visual(
            Box((0.036, 0.018, 0.010)),
            origin=Origin(xyz=(x, 0.0, rail_z - 0.020)),
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

    # Hidden retained insertion of the telescoping posts inside the lower sleeves.
    for i in range(2):
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
            name=f"sliding bushing {i} remains inserted",
        )
        ctx.expect_within(
            upper,
            base,
            axes="xy",
            inner_elem=f"upper_post_{i}",
            outer_elem=f"lower_sleeve_{i}",
            margin=0.001,
            name=f"upper post {i} centered in lower sleeve",
        )
        ctx.expect_overlap(
            upper,
            base,
            axes="z",
            elem_a=f"upper_post_{i}",
            elem_b=f"lower_sleeve_{i}",
            min_overlap=0.16,
            name=f"upper post {i} has collapsed insertion",
        )

    rest_aabb = ctx.part_world_aabb(upper)
    with ctx.pose({height: 0.14}):
        raised_aabb = ctx.part_world_aabb(upper)
        for i in range(2):
            ctx.expect_within(
                upper,
                base,
                axes="xy",
                inner_elem=f"upper_post_{i}",
                outer_elem=f"lower_sleeve_{i}",
                margin=0.001,
                name=f"raised upper post {i} remains centered",
            )
            ctx.expect_overlap(
                upper,
                base,
                axes="z",
                elem_a=f"upper_post_{i}",
                elem_b=f"lower_sleeve_{i}",
                min_overlap=0.025,
                name=f"raised upper post {i} retains insertion",
            )

    ctx.check(
        "height slide raises top rail",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest_aabb={rest_aabb}, raised_aabb={raised_aabb}",
    )

    ctx.expect_overlap(
        upper,
        base,
        axes="x",
        elem_a="rail_outer",
        elem_b="front_base_tube",
        min_overlap=1.35,
        name="outer rail sleeve spans most of wheeled base width",
    )
    ctx.expect_gap(
        upper,
        base,
        axis="z",
        positive_elem="rail_outer",
        negative_elem="front_base_tube",
        min_gap=1.45,
        name="hanging rail is high above base",
    )

    # --- Outer hangers on upper_frame ---
    ctx.check(
        "outer rail has articulated hangers",
        all(
            object_model.get_part(f"top_hanger_{idx}") is not None
            and object_model.get_articulation(f"top_hanger_{idx}_swing") is not None
            for idx in range(4)
        ),
    )
    top_hanger = object_model.get_part("top_hanger_0")
    top_swing = object_model.get_articulation("top_hanger_0_swing")
    ctx.check(
        "outer hanger swings a small range around rail axis",
        top_swing is not None
        and top_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(top_swing.axis) == (1.0, 0.0, 0.0)
        and top_swing.motion_limits is not None
        and top_swing.motion_limits.lower == -0.35
        and top_swing.motion_limits.upper == 0.35,
    )
    # All outer hanger hooks wrap around the outer rail sleeve.
    for idx in range(4):
        hanger_part = object_model.get_part(f"top_hanger_{idx}")
        ctx.allow_overlap(
            upper,
            hanger_part,
            elem_a="rail_outer",
            elem_b="hook_loop",
            reason="The hanger hook intentionally wraps around the outer clothing rail sleeve.",
        )
    ctx.expect_overlap(
        upper,
        top_hanger,
        axes="z",
        elem_a="rail_outer",
        elem_b="hook_loop",
        min_overlap=0.010,
        name="outer hanger hook wraps the outer rail",
    )
    ctx.expect_gap(
        upper,
        top_hanger,
        axis="z",
        positive_elem="rail_outer",
        negative_elem="hanger_body",
        min_gap=0.040,
        name="outer hanger body hangs below outer rail",
    )

    # --- Inner sliding rail and rail_extend prismatic joint ---
    rail_inner_part = object_model.get_part("rail_inner")
    rail_extend = object_model.get_articulation("rail_extend")
    ctx.check(
        "rail_inner part exists with inner tube visual",
        rail_inner_part is not None
        and rail_inner_part.get_visual("inner_tube") is not None,
    )
    ctx.check(
        "rail_extend is a prismatic joint along +X",
        rail_extend is not None
        and rail_extend.articulation_type == ArticulationType.PRISMATIC
        and tuple(rail_extend.axis) == (1.0, 0.0, 0.0)
        and rail_extend.motion_limits is not None
        and rail_extend.motion_limits.lower == 0.0
        and rail_extend.motion_limits.upper == 0.25,
    )

    # The inner tube end cap sits inside the outer sleeve bore near the right rail tee.
    ctx.allow_overlap(
        upper,
        rail_inner_part,
        elem_a="rail_tee_1",
        elem_b="inner_rail_cap",
        reason="The inner tube end cap is intentionally captured inside the outer sleeve bore near the right rail tee junction.",
    )
    ctx.expect_within(
        rail_inner_part,
        upper,
        axes="yz",
        inner_elem="inner_rail_cap",
        outer_elem="rail_outer",
        margin=0.008,
        name="inner rail cap stays within outer sleeve bore",
    )

    # Inner tube stays centered on the outer bore axis (Y/Z) at rest and extended.
    ctx.expect_within(
        rail_inner_part,
        upper,
        axes="yz",
        inner_elem="inner_tube",
        outer_elem="rail_outer",
        margin=0.005,
        name="inner tube centered in outer sleeve bore at rest",
    )
    ctx.expect_overlap(
        rail_inner_part,
        upper,
        axes="x",
        elem_a="inner_tube",
        elem_b="rail_outer",
        min_overlap=0.20,
        name="inner tube retains insertion in outer sleeve at rest",
    )

    # At max extension the inner tube still remains captured inside the outer sleeve.
    rest_inner_aabb = ctx.part_element_world_aabb(rail_inner_part, elem="inner_tube")
    with ctx.pose({rail_extend: 0.25}):
        ext_inner_aabb = ctx.part_element_world_aabb(rail_inner_part, elem="inner_tube")
        ctx.expect_within(
            rail_inner_part,
            upper,
            axes="yz",
            inner_elem="inner_tube",
            outer_elem="rail_outer",
            margin=0.005,
            name="inner tube stays centered in outer sleeve at max extension",
        )
        ctx.expect_overlap(
            rail_inner_part,
            upper,
            axes="x",
            elem_a="inner_tube",
            elem_b="rail_outer",
            min_overlap=0.05,
            name="inner tube retains insertion in outer sleeve at max extension",
        )

    ctx.check(
        "rail_extend moves inner tube outward along +X",
        rest_inner_aabb is not None
        and ext_inner_aabb is not None
        and ext_inner_aabb[1][0] > rest_inner_aabb[1][0] + 0.20,
        details=f"rest={rest_inner_aabb}, extended={ext_inner_aabb}",
    )

    # --- Inner hangers on rail_inner ---
    ctx.check(
        "inner rail has articulated hangers",
        all(
            object_model.get_part(f"inner_hanger_{idx}") is not None
            and object_model.get_articulation(f"inner_hanger_{idx}_swing") is not None
            for idx in range(3)
        ),
    )
    inner_hanger = object_model.get_part("inner_hanger_0")
    inner_swing = object_model.get_articulation("inner_hanger_0_swing")
    ctx.check(
        "inner hanger swings around rail axis",
        inner_swing is not None
        and inner_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(inner_swing.axis) == (1.0, 0.0, 0.0),
    )
    # All inner hanger hooks wrap around the inner sliding rail tube.
    for idx in range(3):
        ih_part = object_model.get_part(f"inner_hanger_{idx}")
        ctx.allow_overlap(
            rail_inner_part,
            ih_part,
            elem_a="inner_tube",
            elem_b="hook_loop",
            reason="The inner hanger hook intentionally wraps around the inner sliding rail tube.",
        )
        # At rest, the inner tube (and its hangers) sit inside the outer sleeve bore,
        # so the hooks pass through the outer sleeve hollow space.
        ctx.allow_overlap(
            upper,
            ih_part,
            elem_a="rail_outer",
            elem_b="hook_loop",
            reason="The inner hanger hook passes through the outer sleeve bore to wrap around the inner tube inside.",
        )
    ctx.expect_overlap(
        rail_inner_part,
        inner_hanger,
        axes="z",
        elem_a="inner_tube",
        elem_b="hook_loop",
        min_overlap=0.008,
        name="inner hanger hook wraps the inner rail",
    )
    ctx.expect_gap(
        rail_inner_part,
        inner_hanger,
        axis="z",
        positive_elem="inner_tube",
        negative_elem="hanger_body",
        min_gap=0.040,
        name="inner hanger body hangs below inner rail",
    )

    return ctx.report()


object_model = build_object_model()
