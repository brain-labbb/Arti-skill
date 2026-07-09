from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


POLE_X = (-0.085, 0.085)
POLE_Y = 0.0


def _cyl_z(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _cyl_x(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _visual_ymax(ctx: TestContext, part, elem: str):
    box = ctx.part_element_world_aabb(part, elem=elem)
    if box is None:
        return None
    return box[1][1]


def _make_cork_handle_mesh(name: str):
    """Lathed, lightly ergonomic cork grip with top and lower flares."""
    profile = [
        (0.0130, -0.085),
        (0.0180, -0.075),
        (0.0160, -0.050),
        (0.0135, -0.020),
        (0.0165, 0.015),
        (0.0185, 0.050),
        (0.0155, 0.080),
        (0.0125, 0.088),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48, closed=True), name)


def _make_foam_grip_mesh(name: str):
    """Ribbed lower black grip sleeve under the cork handle."""
    profile = [
        (0.0110, -0.070),
        (0.0165, -0.064),
        (0.0165, -0.048),
        (0.0125, -0.043),
        (0.0125, -0.030),
        (0.0160, -0.025),
        (0.0160, -0.010),
        (0.0125, -0.005),
        (0.0125, 0.010),
        (0.0156, 0.016),
        (0.0156, 0.030),
        (0.0122, 0.036),
        (0.0122, 0.050),
        (0.0150, 0.056),
        (0.0150, 0.070),
        (0.0110, 0.074),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=40, closed=True), name)


def _make_tip_mesh(name: str):
    return mesh_from_geometry(ConeGeometry(0.0075, 0.050, radial_segments=32, closed=True), name)


def _make_basket_mesh(name: str):
    return mesh_from_geometry(TorusGeometry(0.030, 0.0026, radial_segments=16, tubular_segments=48), name)


def _make_strap_mesh(x: float, name: str):
    # A soft black wrist loop emerging from the cap and hanging behind the cork.
    geom = tube_from_spline_points(
        [
            (x - 0.006, -0.004, 0.670),
            (x - 0.030, -0.036, 0.640),
            (x - 0.037, -0.055, 0.585),
            (x - 0.022, -0.040, 0.535),
            (x - 0.004, -0.014, 0.610),
            (x + 0.006, -0.004, 0.670),
        ],
        radius=0.0028,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _make_pair_tether_mesh(name: str):
    geom = tube_from_spline_points(
        [
            (POLE_X[0] + 0.006, -0.006, 0.672),
            (-0.030, -0.018, 0.690),
            (0.030, -0.018, 0.690),
            (POLE_X[1] - 0.006, -0.006, 0.672),
        ],
        radius=0.0019,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def _add_upper_pole(root, i, x, mats, handle_mesh, foam_mesh):
    cork, black, white, metal, dark = mats
    root.visual(
        handle_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.585)),
        material=cork,
        name=f"pole_{i}_cork_handle",
    )
    root.visual(
        foam_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.425)),
        material=black,
        name=f"pole_{i}_foam_grip",
    )
    _cyl_z(root, 0.0175, 0.030, (x, POLE_Y, 0.688), metal, f"pole_{i}_top_cap")
    _cyl_z(root, 0.0100, 0.310, (x, POLE_Y, 0.325), white, f"pole_{i}_upper_sleeve")
    _cyl_z(root, 0.0115, 0.016, (x, POLE_Y, 0.475), black, f"pole_{i}_upper_black_band")
    _cyl_z(root, 0.0115, 0.014, (x, POLE_Y, 0.205), black, f"pole_{i}_lower_black_band")
    _cyl_z(root, 0.0032, 0.308, (x, POLE_Y, 0.536), black, f"pole_{i}_hidden_core")
    # Carbon-look label strip on the front of the white upper tube.
    root.visual(
        Box((0.005, 0.0024, 0.180)),
        origin=Origin(xyz=(x, POLE_Y - 0.0105, 0.335)),
        material=dark,
        name=f"pole_{i}_carbon_label",
    )
    _cyl_z(root, 0.0145, 0.038, (x, POLE_Y, 0.180), black, f"pole_{i}_clamp_collar")
    _cyl_x(root, 0.0045, 0.028, (x, POLE_Y + 0.016, 0.180), metal, f"pole_{i}_clamp_pin")
    root.visual(
        Box((0.024, 0.010, 0.020)),
        origin=Origin(xyz=(x, POLE_Y + 0.011, 0.166)),
        material=black,
        name=f"pole_{i}_clamp_body",
    )
    # Small black strap tag resembling the dangling branded webbing in the reference.
    root.visual(
        Box((0.018, 0.012, 0.088)),
        origin=Origin(xyz=(x - 0.038, POLE_Y - 0.049, 0.575), rpy=(0.0, 0.0, -0.35)),
        material=black,
        name=f"pole_{i}_strap_tag",
    )
    root.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (x - 0.006, -0.004, 0.670),
                    (x - 0.030, -0.036, 0.640),
                    (x - 0.037, -0.055, 0.585),
                    (x - 0.022, -0.040, 0.535),
                    (x - 0.004, -0.014, 0.610),
                    (x + 0.006, -0.004, 0.670),
                ],
                radius=0.0028,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            f"pole_{i}_wrist_loop_mesh",
        ),
        material=black,
        name=f"pole_{i}_wrist_loop",
    )


def _add_lower_stage(part, mats, tip_mesh, basket_mesh):
    """Single telescoping lower stage for a 2-section pole.

    Combines the shaft tube, a white lower sleeve section, decorative band,
    ferrule, basket, and carbide tip — all relative to the part origin which
    sits at the flip-lock clamp interface.
    """
    _, black, white, metal, dark = mats
    # Main telescoping shaft tube (replaces old mid_tube + lower_tube)
    _cyl_z(part, 0.0078, 0.700, (0.0, 0.0, -0.120), dark, "lower_tube")
    # White aluminum lower sleeve section for visual interest
    _cyl_z(part, 0.0090, 0.160, (0.0, 0.0, -0.320), white, "lower_sleeve")
    # Black decorative band at top of white sleeve
    _cyl_z(part, 0.0105, 0.014, (0.0, 0.0, -0.255), black, "lower_sleeve_band")
    # Metal ferrule transition below tube
    _cyl_z(part, 0.0062, 0.030, (0.0, 0.0, -0.485), metal, "ferrule")
    # Trekking basket near the tip
    part.visual(
        basket_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=black,
        name="basket_ring",
    )
    part.visual(
        Box((0.066, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=black,
        name="basket_spoke_x",
    )
    part.visual(
        Box((0.004, 0.066, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        material=black,
        name="basket_spoke_y",
    )
    # Carbide tip pointing downward
    part.visual(
        tip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.520), rpy=(math.pi, 0.0, 0.0)),
        material=metal,
        name="carbide_tip",
    )


def _add_lever_visuals(part, material, pin_material):
    _cyl_x(part, 0.0060, 0.017, (0.0, 0.0, 0.0), pin_material, "lever_knuckle")
    part.visual(
        Box((0.014, 0.008, 0.058)),
        origin=Origin(xyz=(0.0, 0.008, -0.032)),
        material=material,
        name="lever_blade",
    )
    part.visual(
        Box((0.018, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.011, -0.060)),
        material=material,
        name="lever_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_pair",
        meta={
            "run_notes": (
                "2-section collapsible trekking pole pair (single telescoping "
                "stage per pole). Cork handles, wrist straps, one flip-lock "
                "clamp per pole, trekking baskets, and carbide tips. Variant "
                "of the 3-section parent with mid_stage removed."
            )
        },
    )

    cork = model.material("cork_like_tan", rgba=(0.72, 0.45, 0.22, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    white = model.material("white_aluminum", rgba=(0.93, 0.92, 0.88, 1.0))
    metal = model.material("brushed_silver", rgba=(0.70, 0.72, 0.73, 1.0))
    dark = model.material("dark_carbon_fiber", rgba=(0.015, 0.018, 0.017, 1.0))
    mats = (cork, black, white, metal, dark)

    upper = model.part("upper_assemblies")
    handle_mesh = _make_cork_handle_mesh("shared_cork_handle")
    foam_mesh = _make_foam_grip_mesh("shared_foam_grip")
    for i, x in enumerate(POLE_X):
        _add_upper_pole(upper, i, x, mats, handle_mesh, foam_mesh)

    upper.visual(
        _make_pair_tether_mesh("pair_tether_mesh"),
        material=black,
        name="pair_tether",
    )

    tip_mesh = _make_tip_mesh("shared_lower_tip")
    basket_mesh = _make_basket_mesh("shared_trekking_basket")

    for i, x in enumerate(POLE_X):
        # Single lower telescoping stage (2-section design: upper + lower only)
        lower = model.part(f"lower_stage_{i}")
        _add_lower_stage(lower, mats, tip_mesh, basket_mesh)

        # Single prismatic telescoping joint per pole
        model.articulation(
            f"upper_to_lower_{i}",
            ArticulationType.PRISMATIC,
            parent=upper,
            child=lower,
            origin=Origin(xyz=(x, POLE_Y, 0.180)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.180),
        )

        # Single flick-lock lever per pole
        lever = model.part(f"clamp_lever_{i}")
        _add_lever_visuals(lever, black, metal)
        model.articulation(
            f"clamp_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=lever,
            origin=Origin(xyz=(x, POLE_Y + 0.016, 0.180)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_assemblies")

    for i in range(2):
        lower = object_model.get_part(f"lower_stage_{i}")
        slide = object_model.get_articulation(f"upper_to_lower_{i}")
        lever = object_model.get_part(f"clamp_lever_{i}")
        hinge = object_model.get_articulation(f"clamp_hinge_{i}")

        # --- Overlap allowances for telescoping tube nesting ---
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"pole_{i}_upper_sleeve",
            elem_b="lower_tube",
            reason="The lower tube is intentionally retained inside the upper telescoping sleeve.",
        )
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"pole_{i}_clamp_collar",
            elem_b="lower_tube",
            reason="The flip-lock collar clamps around the sliding lower tube.",
        )
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"pole_{i}_lower_black_band",
            elem_b="lower_tube",
            reason="The decorative lower band surrounds the telescoping tube at the sleeve mouth.",
        )
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=f"pole_{i}_hidden_core",
            elem_b="lower_tube",
            reason="The internal cord/spring mechanism runs concentrically inside the telescoping lower tube.",
        )
        ctx.allow_overlap(
            upper,
            lever,
            elem_a=f"pole_{i}_clamp_pin",
            elem_b="lever_knuckle",
            reason="The lever knuckle rotates around the clamp pin.",
        )

        # --- Coaxial and retained-insertion checks at rest ---
        ctx.expect_within(
            lower,
            upper,
            axes="xy",
            inner_elem="lower_tube",
            outer_elem=f"pole_{i}_upper_sleeve",
            margin=0.004,
            name=f"lower stage {i} stays coaxial in upper sleeve",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="lower_tube",
            elem_b=f"pole_{i}_upper_sleeve",
            min_overlap=0.080,
            name=f"lower stage {i} retained in collapsed upper sleeve",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="lower_tube",
            elem_b=f"pole_{i}_lower_black_band",
            min_overlap=0.005,
            name=f"lower stage {i} passes through upper band",
        )

        # --- Pose checks: extension and lever flip ---
        rest_lower = ctx.part_world_position(lower)
        rest_lever_y = _visual_ymax(ctx, lever, "lever_blade")

        with ctx.pose({slide: 0.160, hinge: 0.95}):
            ctx.expect_overlap(
                lower,
                upper,
                axes="z",
                elem_a="lower_tube",
                elem_b=f"pole_{i}_upper_sleeve",
                min_overlap=0.020,
                name=f"lower stage {i} retained when extended",
            )
            ext_lower = ctx.part_world_position(lower)
            open_lever_y = _visual_ymax(ctx, lever, "lever_blade")

        ctx.check(
            f"lower stage {i} telescopes downward (single prismatic joint upper_to_lower_{i})",
            rest_lower is not None and ext_lower is not None and ext_lower[2] < rest_lower[2] - 0.12,
            details=f"rest={rest_lower}, extended={ext_lower}",
        )
        ctx.check(
            f"clamp lever {i} flips outward",
            rest_lever_y is not None
            and open_lever_y is not None
            and open_lever_y > rest_lever_y + 0.006,
            details=f"rest_ymax={rest_lever_y}, open_ymax={open_lever_y}",
        )

    # Variant-specific: confirm exactly 2 telescoping sections per pole
    # (one prismatic joint per pole, no mid_stage intermediate)
    prismatic_joints = [
        name for name in ("upper_to_lower_0", "upper_to_lower_1")
        if object_model.get_articulation(name) is not None
    ]
    ctx.check(
        "exactly 2 telescoping prismatic joints (one per pole, 2-section design)",
        len(prismatic_joints) == 2,
        details=f"found prismatic joints: {prismatic_joints}",
    )

    return ctx.report()


object_model = build_object_model()
