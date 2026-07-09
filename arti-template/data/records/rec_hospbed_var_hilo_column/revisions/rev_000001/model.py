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
COLUMN_PAINT = Material("column_grey_paint", rgba=(0.72, 0.72, 0.70, 1.0))


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


def _add_caster(part, *, prefix: str, x: float, y: float):
    """Add a hospital-bed swivel caster assembly at the given XY position."""
    _cylinder_y(
        part,
        name=f"{prefix}_tire",
        x=x, y=y, z=0.028,
        length=0.026, radius=0.028,
        material=DARK_RUBBER,
    )
    _cylinder_z(
        part,
        name=f"{prefix}_stem",
        x=x, y=y, z=0.121,
        length=0.082, radius=0.010,
        material=GREY_METAL,
    )
    _cylinder_z(
        part,
        name=f"{prefix}_swivel",
        x=x, y=y, z=0.076,
        length=0.012, radius=0.020,
        material=GREY_METAL,
    )
    part.visual(
        Box((0.046, 0.048, 0.010)),
        origin=_origin(x, y, 0.069),
        material=GREY_METAL,
        name=f"{prefix}_fork_bridge",
    )
    for dy, side in [(-0.018, "0"), (0.018, "1")]:
        part.visual(
            Box((0.024, 0.006, 0.064)),
            origin=_origin(x, y + dy, 0.040),
            material=GREY_METAL,
            name=f"{prefix}_fork_{side}",
        )
    _cylinder_y(
        part,
        name=f"{prefix}_axle",
        x=x, y=y, z=0.028,
        length=0.050, radius=0.0045,
        material=GREY_METAL,
    )
    _cylinder_y(
        part,
        name=f"{prefix}_hub",
        x=x, y=y, z=0.028,
        length=0.028, radius=0.012,
        material=HUB_METAL,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hi_lo_hospital_bed")

    tube_r = 0.018
    bed_len = 2.0
    bed_w = 0.90
    hinge_x = -bed_len / 6.0
    deck_top_z = 0.62

    # ── Pre-build cushion meshes (identical to parent baseline) ──
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

    # ═══════════════════════════════════════════════════════════════════
    # ROOT: cruciform floor base with 4 wheeled arms + outer column
    # ═══════════════════════════════════════════════════════════════════
    base = model.part("base")

    # Central hub plate connecting the 4 cruciform arms to the column.
    _cylinder_z(
        base, name="hub_plate",
        x=0.0, y=0.0, z=0.090,
        length=0.050, radius=0.072,
        material=COLUMN_PAINT,
    )

    # 4 cruciform arms (rectangular tube) radiating from center.
    arm_len = 0.50
    arm_w = 0.060
    arm_h = 0.045
    arm_z = 0.090
    for i, (dx, dy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
        cx = dx * arm_len / 2.0
        cy = dy * arm_len / 2.0
        if dx != 0:
            base.visual(
                Box((arm_len, arm_w, arm_h)),
                origin=_origin(cx, cy, arm_z),
                material=WHITE_STEEL,
                name=f"cruciform_arm_{i}",
            )
        else:
            base.visual(
                Box((arm_w, arm_len, arm_h)),
                origin=_origin(cx, cy, arm_z),
                material=WHITE_STEEL,
                name=f"cruciform_arm_{i}",
            )

    # Caster at each arm tip.
    caster_positions = [
        (arm_len, 0.0, "caster_0"),
        (-arm_len, 0.0, "caster_1"),
        (0.0, arm_len, "caster_2"),
        (0.0, -arm_len, "caster_3"),
    ]
    for cx, cy, prefix in caster_positions:
        _add_caster(base, prefix=prefix, x=cx, y=cy)

    # Outer column tube (stationary housing).
    _cylinder_z(
        base, name="outer_column",
        x=0.0, y=0.0, z=0.325,
        length=0.47, radius=0.050,
        material=COLUMN_PAINT,
    )

    # ═══════════════════════════════════════════════════════════════════
    # LIFT COLUMN: telescoping inner column + deck frame + boards
    # (at q=0 the lift_column frame coincides with the world origin,
    #  so all deck geometry keeps the same local positions as the parent)
    # ═══════════════════════════════════════════════════════════════════
    lift_column = model.part("lift_column")

    # Inner telescoping tube.
    _cylinder_z(
        lift_column, name="inner_column",
        x=0.0, y=0.0, z=0.350,
        length=0.46, radius=0.040,
        material=COLUMN_PAINT,
    )

    # Carriage plate connecting column top to the deck frame side rails.
    lift_column.visual(
        Box((0.32, 0.94, 0.012)),
        origin=_origin(0.0, 0.0, 0.575),
        material=COLUMN_PAINT,
        name="carriage_plate",
    )

    # ── Deck frame (tubular side rails and cross rails) ──
    strut_z_center = (0.36 + 0.585) / 2.0
    strut_len = 0.585 - 0.36
    for y, suffix in [(-0.47, "0"), (0.47, "1")]:
        _cylinder_x(
            lift_column,
            name=f"side_rail_{suffix}",
            x=0.0, y=y, z=0.585,
            length=1.92, radius=tube_r,
            material=WHITE_STEEL,
        )
        _cylinder_x(
            lift_column,
            name=f"lower_side_rail_{suffix}",
            x=0.0, y=y, z=0.36,
            length=2.00, radius=0.014,
            material=WHITE_STEEL,
        )
        for x_pos, s_name in [(-0.60, "a"), (0.60, "b")]:
            _cylinder_z(
                lift_column,
                name=f"rail_strut_{suffix}_{s_name}",
                x=x_pos, y=y, z=strut_z_center,
                length=strut_len, radius=0.012,
                material=WHITE_STEEL,
            )
    for x, suffix in [(-0.96, "head"), (hinge_x, "hinge"), (0.30, "mid"), (0.96, "foot")]:
        _cylinder_y(
            lift_column,
            name=f"{suffix}_cross_rail",
            x=x, y=0.0,
            z=0.560 if suffix in {"head", "hinge"} else 0.585,
            length=0.94, radius=tube_r,
            material=WHITE_STEEL,
        )

    # Fixed sleeping deck panel and mattress.
    lift_column.visual(
        Box((1.28, 0.86, 0.030)),
        origin=_origin(0.34, 0.0, deck_top_z - 0.015),
        material=OFF_WHITE,
        name="fixed_deck",
    )
    lift_column.visual(fixed_mattress, origin=Origin(), material=BLUE_FABRIC, name="fixed_mattress")

    # ── Head and foot boards (shortened posts, no legs to floor) ──
    for x, end in [(-1.00, "head"), (1.00, "foot")]:
        for y, side in [(-0.47, "0"), (0.47, "1")]:
            _cylinder_z(
                lift_column,
                name=f"{end}_post_{side}",
                x=x, y=y, z=0.85,
                length=0.58, radius=0.022,
                material=WHITE_STEEL,
            )
        for z, bar in [(1.13, "top"), (0.92, "middle"), (0.72, "lower")]:
            _cylinder_y(
                lift_column,
                name=f"{end}_{bar}_bar",
                x=x, y=0.0, z=z,
                length=0.94,
                radius=0.020 if bar == "top" else 0.016,
                material=WHITE_STEEL,
            )

    # Hinge knuckles at the backrest division.
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            lift_column,
            name=f"hinge_barrel_{suffix}",
            x=hinge_x, y=y, z=deck_top_z,
            length=0.12, radius=0.018,
            material=WHITE_STEEL,
        )

    # ═══════════════════════════════════════════════════════════════════
    # BACKREST (identical geometry to parent baseline)
    # ═══════════════════════════════════════════════════════════════════
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
        x=0.0, y=0.0, z=0.0,
        length=0.44, radius=0.018,
        material=WHITE_STEEL,
    )
    for y, suffix in [(-0.365, "0"), (0.365, "1")]:
        _cylinder_y(
            backrest,
            name=f"side_hinge_tube_{suffix}",
            x=0.0, y=y, z=0.0,
            length=0.13, radius=0.018,
            material=WHITE_STEEL,
        )
    for y, suffix in [(-0.34, "0"), (0.34, "1")]:
        backrest.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(-0.025, y, -0.006),
            material=WHITE_STEEL,
            name=f"hinge_leaf_{suffix}",
        )

    # ═══════════════════════════════════════════════════════════════════
    # ARTICULATIONS
    # ═══════════════════════════════════════════════════════════════════

    # Hi-lo prismatic lift: column raises/lowers the whole deck on Z.
    model.articulation(
        "column_to_deck",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=_origin(0.0, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.05, lower=0.0, upper=0.30),
    )

    # Backrest hinge (identical to parent baseline, parent is now lift_column).
    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=backrest,
        origin=_origin(hinge_x, 0.0, deck_top_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    backrest = object_model.get_part("backrest")
    lift_joint = object_model.get_articulation("column_to_deck")
    hinge = object_model.get_articulation("base_to_backrest")

    # ── Overall bed scale ──
    lift_aabb = ctx.part_world_aabb(lift_column)
    ctx.check(
        "bed has hospital-bed scale",
        lift_aabb is not None
        and 1.90 <= (lift_aabb[1][0] - lift_aabb[0][0]) <= 2.15
        and 0.85 <= (lift_aabb[1][1] - lift_aabb[0][1]) <= 1.12
        and 0.58 <= lift_aabb[1][2],
        details=f"lift_aabb={lift_aabb}",
    )

    # ── Cruciform base casters ──
    for i in range(4):
        tire = ctx.part_element_world_aabb(base, elem=f"caster_{i}_tire")
        fork_0 = ctx.part_element_world_aabb(base, elem=f"caster_{i}_fork_0")
        fork_1 = ctx.part_element_world_aabb(base, elem=f"caster_{i}_fork_1")
        hub = ctx.part_element_world_aabb(base, elem=f"caster_{i}_hub")

        tire_dims = None
        if tire is not None:
            tire_dims = tuple(tire[1][i] - tire[0][i] for i in range(3))
        ctx.check(
            f"cruciform caster {i} is a vertical wheel at the floor",
            tire is not None
            and tire_dims is not None
            and abs(tire[0][2]) <= 0.002
            and 0.052 <= tire_dims[0] <= 0.060
            and 0.020 <= tire_dims[1] <= 0.032
            and 0.052 <= tire_dims[2] <= 0.060,
            details=f"tire_aabb={tire}, tire_dims={tire_dims}",
        )
        ctx.check(
            f"cruciform caster {i} fork straddles tire",
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
            f"cruciform caster {i} has visible centered hub",
            tire is not None
            and hub is not None
            and tire[0][0] < hub[0][0] < hub[1][0] < tire[1][0]
            and tire[0][2] < hub[0][2] < hub[1][2] < tire[1][2]
            and hub[0][1] < tire[0][1]
            and hub[1][1] > tire[1][1],
            details=f"tire={tire}, hub={hub}",
        )

    # ── Telescoping column fit ──
    ctx.allow_overlap(
        base,
        lift_column,
        elem_a="outer_column",
        elem_b="inner_column",
        reason="The inner telescoping column is intentionally nested inside the outer column housing.",
    )
    ctx.expect_within(
        lift_column, base,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="outer_column",
        margin=0.002,
        name="inner column stays centered in outer column",
    )
    ctx.expect_overlap(
        lift_column, base,
        axes="z",
        elem_a="inner_column",
        elem_b="outer_column",
        min_overlap=0.10,
        name="resting column has retained insertion depth",
    )

    # ── Prismatic lift raises the deck (TARGET-specific assertion) ──
    rest_deck = ctx.part_element_world_aabb(lift_column, elem="fixed_deck")
    with ctx.pose({lift_joint: 0.25}):
        raised_deck = ctx.part_element_world_aabb(lift_column, elem="fixed_deck")
        ctx.check(
            "column_to_deck prismatic raises the deck",
            rest_deck is not None
            and raised_deck is not None
            and raised_deck[0][2] > rest_deck[0][2] + 0.20,
            details=f"rest_deck={rest_deck}, raised_deck={raised_deck}",
        )

    # Extended pose still retains column insertion.
    with ctx.pose({lift_joint: 0.25}):
        ctx.expect_overlap(
            lift_column, base,
            axes="z",
            elem_a="inner_column",
            elem_b="outer_column",
            min_overlap=0.10,
            name="extended column retains insertion depth",
        )

    # ── Backrest / deck relationship (identical to parent baseline) ──
    ctx.expect_overlap(
        backrest, lift_column,
        axes="y",
        min_overlap=0.70,
        elem_a="back_deck",
        elem_b="fixed_deck",
        name="backrest and fixed deck share bed width",
    )
    ctx.expect_gap(
        lift_column, backrest,
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

    return ctx.report()


object_model = build_object_model()
