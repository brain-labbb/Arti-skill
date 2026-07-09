from __future__ import annotations

import math

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
    mesh_from_geometry,
    superellipse_side_loft,
)


WHITE_STEEL = Material("white_powder_coated_steel", rgba=(0.94, 0.94, 0.90, 1.0))
OFF_WHITE = Material("off_white_deck_panels", rgba=(0.86, 0.85, 0.80, 1.0))
BLUE_FABRIC = Material("pale_blue_fabric", rgba=(0.53, 0.68, 0.91, 1.0))
DARK_RUBBER = Material("dark_rubber", rgba=(0.03, 0.03, 0.03, 1.0))
GREY_METAL = Material("caster_grey_metal", rgba=(0.56, 0.56, 0.54, 1.0))
HUB_METAL = Material("caster_hub_metal", rgba=(0.74, 0.74, 0.70, 1.0))


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _cylinder_x(part, *, name: str, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(x, y, z, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, *, name: str, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(x, y, z, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_z(part, *, name: str, x: float, y: float, z: float, length: float, radius: float, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(x, y, z),
        material=material,
        name=name,
    )


def _rounded_cushion_mesh(
    *,
    name: str,
    center_x: float,
    length: float,
    width: float,
    z_min: float,
    z_max: float,
    edge_taper: float,
    softness: float,
):
    """Soft rectangular cushion with rounded long edges and subtly crowned top."""

    half_w = width / 2.0
    sections = [
        (-half_w, z_min + softness, z_max - softness * 0.5, length - edge_taper),
        (-half_w + softness, z_min, z_max, length),
        (0.0, z_min, z_max + softness * 0.35, length),
        (half_w - softness, z_min, z_max, length),
        (half_w, z_min + softness, z_max - softness * 0.5, length - edge_taper),
    ]
    geom = superellipse_side_loft(
        sections,
        exponents=3.2,
        segments=64,
        cap=True,
        closed=True,
    )
    geom.translate(center_x, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_section_hospital_bed")

    tube_r = 0.018
    bed_len = 2.0
    bed_w = 0.90
    hinge_x = -bed_len / 6.0
    deck_top_z = 0.62

    fixed_mattress = _rounded_cushion_mesh(
        name="fixed_mattress",
        center_x=0.34,
        length=1.28,
        width=0.82,
        z_min=deck_top_z - 0.002,
        z_max=deck_top_z + 0.075,
        edge_taper=0.055,
        softness=0.025,
    )
    back_mattress = _rounded_cushion_mesh(
        name="back_mattress",
        center_x=-0.32,
        length=0.60,
        width=0.82,
        z_min=0.0,
        z_max=0.078,
        edge_taper=0.040,
        softness=0.023,
    )
    pillow_mesh = _rounded_cushion_mesh(
        name="pillow",
        center_x=-0.45,
        length=0.42,
        width=0.66,
        z_min=0.083,
        z_max=0.190,
        edge_taper=0.080,
        softness=0.055,
    )

    base = model.part("base_frame")

    # Main powder-coated tubular frame under the sleeping deck.
    for y, suffix in [(-0.47, "0"), (0.47, "1")]:
        _cylinder_x(
            base,
            name=f"side_rail_{suffix}",
            x=0.0,
            y=y,
            z=0.585,
            length=1.92,
            radius=tube_r,
            material=WHITE_STEEL,
        )
        _cylinder_x(
            base,
            name=f"lower_side_rail_{suffix}",
            x=0.0,
            y=y,
            z=0.36,
            length=2.00,
            radius=0.014,
            material=WHITE_STEEL,
        )
    for x, suffix in [(-0.96, "head"), (hinge_x, "hinge"), (0.30, "mid"), (0.96, "foot")]:
        _cylinder_y(
            base,
            name=f"{suffix}_cross_rail",
            x=x,
            y=0.0,
            z=0.560 if suffix in {"head", "hinge"} else 0.585,
            length=0.94,
            radius=tube_r,
            material=WHITE_STEEL,
        )

    # Fixed sleeping deck and fixed mattress section.
    base.visual(
        Box((1.28, 0.86, 0.030)),
        origin=_origin(0.34, 0.0, deck_top_z - 0.015),
        material=OFF_WHITE,
        name="fixed_deck",
    )
    base.visual(fixed_mattress, origin=Origin(), material=BLUE_FABRIC, name="fixed_mattress")

    # Low tubular head and foot boards, using the corner posts as the bed legs.
    for x, end in [(-1.00, "head"), (1.00, "foot")]:
        for y, side in [(-0.47, "0"), (0.47, "1")]:
            _cylinder_z(
                base,
                name=f"{end}_post_{side}",
                x=x,
                y=y,
                z=0.62,
                length=1.08,
                radius=0.022,
                material=WHITE_STEEL,
            )
        for z, bar in [(1.13, "top"), (0.92, "middle"), (0.72, "lower")]:
            _cylinder_y(
                base,
                name=f"{end}_{bar}_bar",
                x=x,
                y=0.0,
                z=z,
                length=0.94,
                radius=0.020 if bar == "top" else 0.016,
                material=WHITE_STEEL,
            )

    # Four identical hospital-bed swivel casters: upright rolling wheel in a fork yoke,
    # a visible hub/axle, and a compact swivel barrel with a stem into the white leg.
    for x, y, suffix in [
        (-1.00, -0.47, "0"),
        (-1.00, 0.47, "1"),
        (1.00, -0.47, "2"),
        (1.00, 0.47, "3"),
    ]:
        _cylinder_y(
            base,
            name=f"caster_tire_{suffix}",
            x=x,
            y=y,
            z=0.028,
            length=0.026,
            radius=0.028,
            material=DARK_RUBBER,
        )
        _cylinder_z(
            base,
            name=f"caster_stem_{suffix}",
            x=x,
            y=y,
            z=0.121,
            length=0.082,
            radius=0.010,
            material=GREY_METAL,
        )
        _cylinder_z(
            base,
            name=f"caster_swivel_{suffix}",
            x=x,
            y=y,
            z=0.076,
            length=0.012,
            radius=0.020,
            material=GREY_METAL,
        )
        base.visual(
            Box((0.046, 0.048, 0.010)),
            origin=_origin(x, y, 0.069),
            material=GREY_METAL,
            name=f"caster_fork_bridge_{suffix}",
        )
        for dy, side in [(-0.018, "0"), (0.018, "1")]:
            base.visual(
                Box((0.024, 0.006, 0.064)),
                origin=_origin(x, y + dy, 0.040),
                material=GREY_METAL,
                name=f"caster_fork_{suffix}_{side}",
            )
        _cylinder_y(
            base,
            name=f"caster_axle_{suffix}",
            x=x,
            y=y,
            z=0.028,
            length=0.050,
            radius=0.0045,
            material=GREY_METAL,
        )
        _cylinder_y(
            base,
            name=f"caster_hub_{suffix}",
            x=x,
            y=y,
            z=0.028,
            length=0.028,
            radius=0.012,
            material=HUB_METAL,
        )

    # Fixed hinge knuckles at the division between flat section and backrest.
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            base,
            name=f"hinge_barrel_{suffix}",
            x=hinge_x,
            y=y,
            z=deck_top_z,
            length=0.12,
            radius=0.018,
            material=WHITE_STEEL,
        )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.63, 0.86, 0.030)),
        origin=_origin(-0.315, 0.0, -0.015),
        material=OFF_WHITE,
        name="back_deck",
    )
    backrest.visual(back_mattress, origin=Origin(), material=BLUE_FABRIC, name="back_mattress")
    backrest.visual(pillow_mesh, origin=Origin(), material=BLUE_FABRIC, name="pillow")
    _cylinder_y(
        backrest,
        name="backrest_hinge_tube",
        x=0.0,
        y=0.0,
        z=0.0,
        length=0.44,
        radius=0.018,
        material=WHITE_STEEL,
    )
    for y, suffix in [(-0.365, "0"), (0.365, "1")]:
        _cylinder_y(
            backrest,
            name=f"side_hinge_tube_{suffix}",
            x=0.0,
            y=y,
            z=0.0,
            length=0.13,
            radius=0.018,
            material=WHITE_STEEL,
        )
    for y, suffix in [(-0.34, "0"), (0.34, "1")]:
        backrest.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(-0.025, y, -0.006),
            material=WHITE_STEEL,
            name=f"hinge_leaf_{suffix}",
        )

    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=_origin(hinge_x, 0.0, deck_top_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.15),
    )

    # --- Drop-down safety side rails (full-length tubular guards) ---
    rail_length = 1.50
    rail_guard_height = 0.30
    rail_tube_r = 0.012
    rail_guard_r = 0.014
    stanchion_xs = [-0.60, -0.20, 0.20, 0.60]
    pivot_z = 0.59  # just above the base frame side rail center

    for side_name, y_sign, axis_vec in [
        ("left", -1.0, (1.0, 0.0, 0.0)),
        ("right", 1.0, (-1.0, 0.0, 0.0)),
    ]:
        pivot_y = y_sign * 0.50  # outer face of the base side rail

        rail = model.part(f"side_rail_{side_name}")

        # Top guard bar — the main horizontal tube the patient/caregiver grips
        _cylinder_x(
            rail,
            name="guard_bar",
            x=0.0, y=0.0, z=rail_guard_height,
            length=rail_length, radius=rail_guard_r, material=WHITE_STEEL,
        )

        # Middle horizontal bar for intermediate retention
        _cylinder_x(
            rail,
            name="mid_bar",
            x=0.0, y=0.0, z=rail_guard_height * 0.5,
            length=rail_length, radius=rail_tube_r, material=WHITE_STEEL,
        )

        # Bottom pivot tube — rotates against the base frame side rail
        _cylinder_x(
            rail,
            name="pivot_tube",
            x=0.0, y=0.0, z=0.0,
            length=rail_length, radius=rail_tube_r, material=WHITE_STEEL,
        )

        # Vertical stanchions connecting the three horizontal tubes
        for i, sx in enumerate(stanchion_xs):
            _cylinder_z(
                rail,
                name=f"stanchion_{i}",
                x=sx, y=0.0, z=rail_guard_height / 2.0,
                length=rail_guard_height, radius=rail_tube_r, material=WHITE_STEEL,
            )

        # Pivot latch brackets (small plates showing the drop-down mount)
        for i, sx in enumerate(stanchion_xs[::2]):  # two brackets per rail
            rail.visual(
                Box((0.040, 0.028, 0.040)),
                origin=_origin(sx, 0.0, -0.005),
                material=GREY_METAL,
                name=f"pivot_latch_{i}",
            )

        model.articulation(
            f"base_to_side_rail_{side_name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=rail,
            origin=_origin(0.0, pivot_y, pivot_z),
            axis=axis_vec,
            motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.57),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    hinge = object_model.get_articulation("base_to_backrest")

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "bed has hospital-bed scale",
        base_aabb is not None
        and 1.90 <= (base_aabb[1][0] - base_aabb[0][0]) <= 2.15
        and 0.85 <= (base_aabb[1][1] - base_aabb[0][1]) <= 1.12
        and 0.58 <= base_aabb[1][2],
        details=f"base_aabb={base_aabb}",
    )
    for suffix in ("0", "1", "2", "3"):
        tire = ctx.part_element_world_aabb(base, elem=f"caster_tire_{suffix}")
        fork_0 = ctx.part_element_world_aabb(base, elem=f"caster_fork_{suffix}_0")
        fork_1 = ctx.part_element_world_aabb(base, elem=f"caster_fork_{suffix}_1")
        hub = ctx.part_element_world_aabb(base, elem=f"caster_hub_{suffix}")

        tire_dims = None
        if tire is not None:
            tire_dims = tuple(tire[1][i] - tire[0][i] for i in range(3))
        ctx.check(
            f"caster {suffix} is a vertical wheel at the floor",
            tire is not None
            and tire_dims is not None
            and abs(tire[0][2]) <= 0.002
            and 0.052 <= tire_dims[0] <= 0.060
            and 0.020 <= tire_dims[1] <= 0.032
            and 0.052 <= tire_dims[2] <= 0.060,
            details=f"tire_aabb={tire}, tire_dims={tire_dims}",
        )
        ctx.check(
            f"caster {suffix} fork straddles tire",
            tire is not None
            and fork_0 is not None
            and fork_1 is not None
            and fork_0[1][1] <= tire[0][1] - 0.001
            and fork_1[0][1] >= tire[1][1] + 0.001
            and fork_0[0][2] <= (tire[0][2] + tire[1][2]) / 2.0 <= fork_0[1][2]
            and fork_1[0][2] <= (tire[0][2] + tire[1][2]) / 2.0 <= fork_1[1][2],
            details=f"tire={tire}, fork_0={fork_0}, fork_1={fork_1}",
        )
        ctx.check(
            f"caster {suffix} has visible centered hub",
            tire is not None
            and hub is not None
            and tire[0][0] < hub[0][0] < hub[1][0] < tire[1][0]
            and tire[0][2] < hub[0][2] < hub[1][2] < tire[1][2]
            and hub[0][1] < tire[0][1]
            and hub[1][1] > tire[1][1],
            details=f"tire={tire}, hub={hub}",
        )
    ctx.expect_overlap(
        backrest,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="back_deck",
        elem_b="fixed_deck",
        name="backrest and fixed deck share bed width",
    )
    ctx.expect_gap(
        base,
        backrest,
        axis="x",
        min_gap=0.0,
        max_gap=0.040,
        positive_elem="fixed_deck",
        negative_elem="back_deck",
        name="flat deck sections meet at hinge with small gap",
    )

    closed_aabb = ctx.part_element_world_aabb(backrest, elem="pillow")
    with ctx.pose({hinge: 0.95}):
        raised_aabb = ctx.part_element_world_aabb(backrest, elem="pillow")
        ctx.check(
            "backrest raises pillow upward",
            closed_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > closed_aabb[1][2] + 0.35,
            details=f"closed={closed_aabb}, raised={raised_aabb}",
        )

    # --- Side rail tests ---
    for side in ("left", "right"):
        rail_part = object_model.get_part(f"side_rail_{side}")
        rail_joint = object_model.get_articulation(f"base_to_side_rail_{side}")

        # Rail spans most of the bed length (full-length guard)
        rail_aabb = ctx.part_world_aabb(rail_part)
        ctx.check(
            f"side_rail_{side} spans most of the bed length",
            rail_aabb is not None and (rail_aabb[1][0] - rail_aabb[0][0]) >= 1.40,
            details=f"rail_aabb={rail_aabb}",
        )

        # Guard bar is well above the deck when raised (q=0)
        guard_up = ctx.part_element_world_aabb(rail_part, elem="guard_bar")
        ctx.check(
            f"side_rail_{side} guard bar is above deck when raised",
            guard_up is not None and guard_up[0][2] > 0.80,
            details=f"guard_bar_up={guard_up}",
        )

        # Pivot tube stays near the base frame side rail
        pivot_aabb = ctx.part_element_world_aabb(rail_part, elem="pivot_tube")
        ctx.check(
            f"side_rail_{side} pivot tube sits at base frame rail height",
            pivot_aabb is not None
            and 0.56 <= pivot_aabb[0][2] <= 0.62
            and 0.56 <= pivot_aabb[1][2] <= 0.62,
            details=f"pivot_aabb={pivot_aabb}",
        )

        # Drop-down motion: guard bar moves significantly downward
        with ctx.pose({rail_joint: 1.57}):
            guard_down = ctx.part_element_world_aabb(rail_part, elem="guard_bar")
            ctx.check(
                f"side_rail_{side} drops down when articulated",
                guard_up is not None
                and guard_down is not None
                and guard_down[1][2] < guard_up[0][2],
                details=f"raised={guard_up}, dropped={guard_down}",
            )

    return ctx.report()


object_model = build_object_model()
