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
    model = ArticulatedObject(name="two_section_profiling_hospital_bed")

    tube_r = 0.018
    bed_len = 2.0
    bed_w = 0.90
    backrest_hinge_x = -bed_len / 6.0  # -0.333
    knee_hinge_x = 0.30
    deck_top_z = 0.62

    # --- Cushion meshes (precomputed for reuse) ---

    # Hip/seat mattress: fixed section between backrest hinge and knee hinge.
    hip_center_x = (backrest_hinge_x + knee_hinge_x) / 2.0  # ~-0.017
    hip_section_len = knee_hinge_x - backrest_hinge_x  # ~0.633
    hip_mattress = _rounded_cushion_mesh(
        name="hip_mattress",
        center_x=hip_center_x,
        length=hip_section_len - 0.06,
        width=0.82,
        z_min=deck_top_z - 0.002,
        z_max=deck_top_z + 0.075,
        edge_taper=0.055,
        softness=0.025,
    )

    # Backrest mattress (unchanged from parent baseline).
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

    # Knee section mattress (relative to knee_section part frame at hinge line).
    knee_section_len = 0.98 - knee_hinge_x  # 0.68
    knee_mattress = _rounded_cushion_mesh(
        name="knee_mattress",
        center_x=knee_section_len / 2.0,
        length=knee_section_len - 0.06,
        width=0.82,
        z_min=-0.002,
        z_max=0.075,
        edge_taper=0.050,
        softness=0.023,
    )

    # ====================== BASE FRAME ======================
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
    for x, suffix in [
        (-0.96, "head"),
        (backrest_hinge_x, "backrest_hinge"),
        (knee_hinge_x, "knee_hinge"),
        (0.96, "foot"),
    ]:
        _cylinder_y(
            base,
            name=f"{suffix}_cross_rail",
            x=x,
            y=0.0,
            z=0.555,
            length=0.94,
            radius=tube_r,
            material=WHITE_STEEL,
        )

    # Fixed hip/seat deck panel (between backrest hinge and knee hinge).
    hip_deck_len = hip_section_len - 0.03
    base.visual(
        Box((hip_deck_len, 0.86, 0.030)),
        origin=_origin(hip_center_x, 0.0, deck_top_z - 0.015),
        material=OFF_WHITE,
        name="hip_deck",
    )
    base.visual(hip_mattress, origin=Origin(), material=BLUE_FABRIC, name="hip_mattress")

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

    # Four identical hospital-bed swivel casters.
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

    # Backrest hinge barrels on the base frame.
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            base,
            name=f"backrest_hinge_barrel_{suffix}",
            x=backrest_hinge_x,
            y=y,
            z=deck_top_z,
            length=0.12,
            radius=0.018,
            material=WHITE_STEEL,
        )

    # Knee hinge barrels on the base frame.
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            base,
            name=f"knee_hinge_barrel_{suffix}",
            x=knee_hinge_x,
            y=y,
            z=deck_top_z,
            length=0.12,
            radius=0.018,
            material=WHITE_STEEL,
        )

    # ====================== BACKREST ======================
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
            name=f"backrest_side_hinge_tube_{suffix}",
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
            name=f"backrest_hinge_leaf_{suffix}",
        )

    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=_origin(backrest_hinge_x, 0.0, deck_top_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.15),
    )

    # ====================== KNEE SECTION ======================
    knee = model.part("knee_section")
    knee.visual(
        Box((knee_section_len, 0.86, 0.030)),
        origin=_origin(knee_section_len / 2.0, 0.0, -0.015),
        material=OFF_WHITE,
        name="knee_deck",
    )
    knee.visual(knee_mattress, origin=Origin(), material=BLUE_FABRIC, name="knee_mattress")

    # Knee hinge tube and leaves (at the knee_section frame = hinge line).
    _cylinder_y(
        knee,
        name="knee_hinge_tube",
        x=0.0,
        y=0.0,
        z=0.0,
        length=0.44,
        radius=0.018,
        material=WHITE_STEEL,
    )
    for y, suffix in [(-0.365, "0"), (0.365, "1")]:
        _cylinder_y(
            knee,
            name=f"knee_side_hinge_tube_{suffix}",
            x=0.0,
            y=y,
            z=0.0,
            length=0.13,
            radius=0.018,
            material=WHITE_STEEL,
        )
    for y, suffix in [(-0.34, "0"), (0.34, "1")]:
        knee.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(0.025, y, -0.006),
            material=WHITE_STEEL,
            name=f"knee_hinge_leaf_{suffix}",
        )

    # Knee section articulation: axis (0, -1, 0) so positive q raises the
    # foot end (+X) upward (+Z) to bend the patient's knees.
    model.articulation(
        "base_to_knee_section",
        ArticulationType.REVOLUTE,
        parent=base,
        child=knee,
        origin=_origin(knee_hinge_x, 0.0, deck_top_z),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    knee = object_model.get_part("knee_section")
    backrest_hinge = object_model.get_articulation("base_to_backrest")
    knee_hinge = object_model.get_articulation("base_to_knee_section")

    # --- Scale check ---
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "bed has hospital-bed scale",
        base_aabb is not None
        and 1.90 <= (base_aabb[1][0] - base_aabb[0][0]) <= 2.15
        and 0.85 <= (base_aabb[1][1] - base_aabb[0][1]) <= 1.12
        and 0.58 <= base_aabb[1][2],
        details=f"base_aabb={base_aabb}",
    )

    # --- Caster checks (unchanged from parent) ---
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

    # --- Backrest checks (unchanged from parent) ---
    ctx.expect_overlap(
        backrest,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="back_deck",
        elem_b="hip_deck",
        name="backrest and hip deck share bed width",
    )
    ctx.expect_gap(
        base,
        backrest,
        axis="x",
        min_gap=0.0,
        max_gap=0.040,
        positive_elem="hip_deck",
        negative_elem="back_deck",
        name="hip deck meets backrest at hinge with small gap",
    )

    closed_pillow = ctx.part_element_world_aabb(backrest, elem="pillow")
    with ctx.pose({backrest_hinge: 0.95}):
        raised_pillow = ctx.part_element_world_aabb(backrest, elem="pillow")
        ctx.check(
            "backrest raises pillow upward",
            closed_pillow is not None
            and raised_pillow is not None
            and raised_pillow[1][2] > closed_pillow[1][2] + 0.35,
            details=f"closed={closed_pillow}, raised={raised_pillow}",
        )

    # --- Knee section checks (new for two-section variant) ---
    ctx.expect_overlap(
        knee,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="knee_deck",
        elem_b="hip_deck",
        name="knee section and hip deck share bed width",
    )
    ctx.expect_gap(
        knee,
        base,
        axis="x",
        min_gap=-0.005,
        max_gap=0.040,
        positive_elem="knee_deck",
        negative_elem="hip_deck",
        name="knee deck meets hip deck at knee hinge with small gap",
    )

    # Prove the knee section articulation raises the foot end upward.
    closed_knee_tip = ctx.part_element_world_aabb(knee, elem="knee_deck")
    with ctx.pose({knee_hinge: 0.55}):
        raised_knee_tip = ctx.part_element_world_aabb(knee, elem="knee_deck")
        ctx.check(
            "knee_section raises foot end upward via base_to_knee_section",
            closed_knee_tip is not None
            and raised_knee_tip is not None
            and raised_knee_tip[1][2] > closed_knee_tip[1][2] + 0.15,
            details=f"closed={closed_knee_tip}, raised={raised_knee_tip}",
        )

    return ctx.report()


object_model = build_object_model()
