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


def _make_twist_collar_mesh(name: str, inner_r: float, outer_r: float, length: float):
    """Knurled twist-lock expander collar ring with grip ridges on the outer surface."""
    h = length / 2.0
    ridge = 0.0008
    # Profile traces the annular cross-section: inner bottom → outer bottom
    # (with grip ridges) → outer top → inner top → close back through the bore.
    profile = [
        (inner_r, -h),
        (outer_r, -h),
        (outer_r + ridge, -h * 0.45),
        (outer_r, 0.0),
        (outer_r + ridge, h * 0.45),
        (outer_r, h),
        (inner_r, h),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=36, closed=True), name)


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
    cork, black, blue, metal, dark = mats
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
    _cyl_z(root, 0.0100, 0.310, (x, POLE_Y, 0.325), blue, f"pole_{i}_upper_sleeve")
    _cyl_z(root, 0.0115, 0.016, (x, POLE_Y, 0.475), black, f"pole_{i}_upper_black_band")
    _cyl_z(root, 0.0115, 0.014, (x, POLE_Y, 0.205), black, f"pole_{i}_lower_black_band")
    _cyl_z(root, 0.0032, 0.308, (x, POLE_Y, 0.536), black, f"pole_{i}_hidden_core")
    # Carbon-look label strip on the front of the blue upper tube.
    root.visual(
        Box((0.005, 0.0024, 0.180)),
        origin=Origin(xyz=(x, POLE_Y - 0.0105, 0.335)),
        material=dark,
        name=f"pole_{i}_carbon_label",
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


def _add_mid_stage(part, i, mats):
    _, black, blue, metal, dark = mats
    _cyl_z(part, 0.0065, 0.540, (0.0, 0.0, -0.075), dark, "mid_tube")
    _cyl_z(part, 0.0086, 0.180, (0.0, 0.0, -0.265), blue, "lower_sleeve")
    _cyl_z(part, 0.0100, 0.014, (0.0, 0.0, -0.198), black, "lower_sleeve_band")


def _add_lower_stage(part, mats, tip_mesh, basket_mesh):
    _, black, _, metal, dark = mats
    _cyl_z(part, 0.0050, 0.480, (0.0, 0.0, -0.110), dark, "lower_tube")
    _cyl_z(part, 0.0062, 0.030, (0.0, 0.0, -0.325), metal, "ferrule")
    part.visual(
        basket_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=black,
        name="basket_ring",
    )
    part.visual(Box((0.066, 0.004, 0.004)), origin=Origin(xyz=(0.0, 0.0, -0.270)), material=black, name="basket_spoke_x")
    part.visual(Box((0.004, 0.066, 0.004)), origin=Origin(xyz=(0.0, 0.0, -0.270)), material=black, name="basket_spoke_y")
    part.visual(
        tip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.362), rpy=(math.pi, 0.0, 0.0)),
        material=metal,
        name="carbide_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="collapsible_trekking_pole_pair",
        meta={
            "run_notes": (
                "Twist-lock expander variant of the collapsible trekking pole pair. "
                "External flip-lock levers replaced with internal twist-lock expander "
                "collars at each telescoping section boundary. Each collar rotates "
                "about the shaft axis via a CONTINUOUS joint to lock/unlock the "
                "telescoping section. Blue anodized aluminum shaft colorway."
            )
        },
    )

    cork = model.material("cork_like_tan", rgba=(0.72, 0.45, 0.22, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    blue = model.material("blue_anodized_aluminum", rgba=(0.10, 0.28, 0.55, 1.0))
    metal = model.material("brushed_silver", rgba=(0.70, 0.72, 0.73, 1.0))
    dark = model.material("dark_carbon_fiber", rgba=(0.015, 0.018, 0.017, 1.0))
    collar_blue = model.material("collar_anodized_blue", rgba=(0.06, 0.18, 0.42, 1.0))
    mats = (cork, black, blue, metal, dark)

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

    # Shared twist collar meshes (one size per telescoping boundary).
    # Inner bore is slightly smaller than the parent sleeve outer radius so
    # the collar physically wraps around the sleeve mouth (seated contact).
    upper_collar_mesh = _make_twist_collar_mesh(
        "upper_twist_collar_mesh", inner_r=0.0097, outer_r=0.0140, length=0.030,
    )
    lower_collar_mesh = _make_twist_collar_mesh(
        "lower_twist_collar_mesh", inner_r=0.0083, outer_r=0.0122, length=0.026,
    )

    for i, x in enumerate(POLE_X):
        mid = model.part(f"mid_stage_{i}")
        _add_mid_stage(mid, i, mats)
        lower = model.part(f"lower_stage_{i}")
        _add_lower_stage(lower, mats, tip_mesh, basket_mesh)

        # --- Prismatic telescoping joints (unchanged from parent) ---
        model.articulation(
            f"upper_to_mid_{i}",
            ArticulationType.PRISMATIC,
            parent=upper,
            child=mid,
            origin=Origin(xyz=(x, POLE_Y, 0.180)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.160),
        )
        model.articulation(
            f"mid_to_lower_{i}",
            ArticulationType.PRISMATIC,
            parent=mid,
            child=lower,
            origin=Origin(xyz=(0.0, 0.0, -0.290)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=60.0, velocity=0.22, lower=0.0, upper=0.140),
        )

        # --- Upper twist-lock expander collar ---
        # Sits at the upper sleeve mouth; rotates about shaft Z to lock/unlock.
        upper_collar = model.part(f"upper_twist_collar_{i}")
        upper_collar.visual(
            upper_collar_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=collar_blue,
            name="collar_ring",
        )
        model.articulation(
            f"upper_twist_{i}",
            ArticulationType.CONTINUOUS,
            parent=upper,
            child=upper_collar,
            origin=Origin(xyz=(x, POLE_Y, 0.170)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0),
        )

        # --- Lower twist-lock expander collar ---
        # Sits at the lower sleeve mouth on the mid stage; rotates about shaft Z.
        lower_collar = model.part(f"lower_twist_collar_{i}")
        lower_collar.visual(
            lower_collar_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=collar_blue,
            name="collar_ring",
        )
        model.articulation(
            f"lower_twist_{i}",
            ArticulationType.CONTINUOUS,
            parent=mid,
            child=lower_collar,
            origin=Origin(xyz=(0.0, 0.0, -0.355)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_assemblies")

    for i in range(2):
        mid = object_model.get_part(f"mid_stage_{i}")
        lower = object_model.get_part(f"lower_stage_{i}")
        upper_slide = object_model.get_articulation(f"upper_to_mid_{i}")
        lower_slide = object_model.get_articulation(f"mid_to_lower_{i}")
        upper_collar = object_model.get_part(f"upper_twist_collar_{i}")
        lower_collar = object_model.get_part(f"lower_twist_collar_{i}")
        upper_twist = object_model.get_articulation(f"upper_twist_{i}")
        lower_twist = object_model.get_articulation(f"lower_twist_{i}")

        # --- Telescoping sleeve intentional overlaps ---
        ctx.allow_overlap(
            upper, mid,
            elem_a=f"pole_{i}_upper_sleeve", elem_b="mid_tube",
            reason="The mid tube is intentionally shown retained inside the upper telescoping sleeve.",
        )
        ctx.allow_overlap(
            upper, mid,
            elem_a=f"pole_{i}_lower_black_band", elem_b="mid_tube",
            reason="The decorative lower band surrounds the telescoping mid tube at the sleeve mouth.",
        )
        ctx.allow_overlap(
            mid, lower,
            elem_a="lower_sleeve", elem_b="lower_tube",
            reason="The lower tube is intentionally inserted into the lower telescoping sleeve.",
        )
        ctx.allow_overlap(
            mid, lower,
            elem_a="lower_sleeve_band", elem_b="lower_tube",
            reason="The black lower sleeve band encircles the inserted lower telescoping tube.",
        )
        ctx.allow_overlap(
            mid, lower,
            elem_a="mid_tube", elem_b="lower_tube",
            reason="The lower carbon tube is intentionally shown telescoping inside the hollow mid-stage tube proxy.",
        )

        # --- Twist collar seating overlaps ---
        # The collar ring seats on the outside of the sleeve mouth and encircles
        # the inner tube passing through.
        ctx.allow_overlap(
            upper, upper_collar,
            elem_a=f"pole_{i}_upper_sleeve", elem_b="collar_ring",
            reason="The upper twist collar seats on the outside of the upper sleeve mouth.",
        )
        ctx.allow_overlap(
            upper_collar, mid,
            elem_a="collar_ring", elem_b="mid_tube",
            reason="The upper twist collar encircles the mid tube passing through the sleeve mouth.",
        )
        ctx.allow_overlap(
            mid, lower_collar,
            elem_a="lower_sleeve", elem_b="collar_ring",
            reason="The lower twist collar seats on the outside of the lower sleeve mouth.",
        )
        ctx.allow_overlap(
            lower_collar, lower,
            elem_a="collar_ring", elem_b="lower_tube",
            reason="The lower twist collar encircles the lower tube passing through the sleeve mouth.",
        )

        # --- Telescoping alignment and retention ---
        ctx.expect_within(
            mid, upper, axes="xy",
            inner_elem="mid_tube", outer_elem=f"pole_{i}_upper_sleeve",
            margin=0.004,
            name=f"mid stage {i} stays coaxial in upper sleeve",
        )
        ctx.expect_overlap(
            mid, upper, axes="z",
            elem_a="mid_tube", elem_b=f"pole_{i}_upper_sleeve",
            min_overlap=0.045,
            name=f"mid stage {i} retained in collapsed upper sleeve",
        )
        ctx.expect_overlap(
            mid, upper, axes="z",
            elem_a="mid_tube", elem_b=f"pole_{i}_lower_black_band",
            min_overlap=0.005,
            name=f"mid stage {i} passes through upper band",
        )
        ctx.expect_within(
            lower, mid, axes="xy",
            inner_elem="lower_tube", outer_elem="lower_sleeve",
            margin=0.004,
            name=f"lower stage {i} stays coaxial in lower sleeve",
        )
        ctx.expect_overlap(
            lower, mid, axes="z",
            elem_a="lower_tube", elem_b="lower_sleeve",
            min_overlap=0.070,
            name=f"lower stage {i} retained in collapsed lower sleeve",
        )
        ctx.expect_overlap(
            lower, mid, axes="z",
            elem_a="lower_tube", elem_b="lower_sleeve_band",
            min_overlap=0.005,
            name=f"lower stage {i} passes through lower sleeve band",
        )
        ctx.expect_overlap(
            lower, mid, axes="z",
            elem_a="lower_tube", elem_b="mid_tube",
            min_overlap=0.050,
            name=f"lower stage {i} remains inside mid tube",
        )

        # --- Twist collar seating and joint type ---
        ctx.expect_contact(
            upper_collar, upper,
            elem_a="collar_ring", elem_b=f"pole_{i}_upper_sleeve",
            contact_tol=0.005,
            name=f"upper twist collar {i} seated on upper sleeve mouth",
        )
        ctx.expect_contact(
            lower_collar, mid,
            elem_a="collar_ring", elem_b="lower_sleeve",
            contact_tol=0.005,
            name=f"lower twist collar {i} seated on lower sleeve mouth",
        )
        ctx.check(
            f"upper_twist_{i} is CONTINUOUS joint",
            upper_twist.articulation_type == ArticulationType.CONTINUOUS,
            details=f"expected CONTINUOUS, got {upper_twist.articulation_type}",
        )
        ctx.check(
            f"lower_twist_{i} is CONTINUOUS joint",
            lower_twist.articulation_type == ArticulationType.CONTINUOUS,
            details=f"expected CONTINUOUS, got {lower_twist.articulation_type}",
        )

        # --- Telescoping motion and twist collar rotation ---
        rest_mid = ctx.part_world_position(mid)
        rest_lower = ctx.part_world_position(lower)

        with ctx.pose({upper_slide: 0.140, lower_slide: 0.120, upper_twist: 1.2, lower_twist: 1.2}):
            ctx.expect_overlap(
                mid, upper, axes="z",
                elem_a="mid_tube", elem_b=f"pole_{i}_upper_sleeve",
                min_overlap=0.010,
                name=f"mid stage {i} retained when extended",
            )
            ctx.expect_overlap(
                lower, mid, axes="z",
                elem_a="lower_tube", elem_b="lower_sleeve",
                min_overlap=0.010,
                name=f"lower stage {i} retained when extended",
            )
            ext_mid = ctx.part_world_position(mid)
            ext_lower = ctx.part_world_position(lower)

        ctx.check(
            f"mid stage {i} telescopes downward",
            rest_mid is not None and ext_mid is not None and ext_mid[2] < rest_mid[2] - 0.10,
            details=f"rest={rest_mid}, extended={ext_mid}",
        )
        ctx.check(
            f"lower stage {i} telescopes downward",
            rest_lower is not None and ext_lower is not None and ext_lower[2] < rest_lower[2] - 0.20,
            details=f"rest={rest_lower}, extended={ext_lower}",
        )

    return ctx.report()


object_model = build_object_model()
