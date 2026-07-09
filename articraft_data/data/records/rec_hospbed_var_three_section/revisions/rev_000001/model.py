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


def _add_hinge_hardware(
    part,
    *,
    hinge_x_local: float,
    tube_length: float,
    leaf_half_span: float,
    barrel_y_positions: tuple[float, ...],
    leaf_y_positions: tuple[float, ...],
    prefix: str,
):
    """Add hinge tube and leaves to a child part at its proximal hinge line."""
    _cylinder_y(
        part,
        name=f"{prefix}_hinge_tube",
        x=hinge_x_local,
        y=0.0,
        z=0.0,
        length=tube_length,
        radius=0.018,
        material=WHITE_STEEL,
    )
    for i, y in enumerate(leaf_y_positions):
        part.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(hinge_x_local - 0.025, y, -0.006),
            material=WHITE_STEEL,
            name=f"{prefix}_hinge_leaf_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_section_hospital_bed")

    tube_r = 0.018
    bed_len = 2.0
    bed_w = 0.90
    deck_top_z = 0.62

    # Hinge positions along X (head = -X, foot = +X)
    backrest_hinge_x = -bed_len / 6.0  # ≈ -0.333
    thigh_hinge_x = 0.10
    thigh_length = 0.45
    calf_hinge_x_local = thigh_length  # in thigh's local frame
    calf_length = 0.42

    # --- Pre-build cushion meshes ---

    # Pillow and backrest mattress sit on the backrest part (local coords).
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

    # Hip mattress on the base frame (world coords).
    # Hip deck ends before the thigh hinge line to clear the hinge tube.
    hip_deck_start = backrest_hinge_x + 0.033  # ≈ -0.30
    hip_deck_end = thigh_hinge_x - 0.022  # ≈ 0.078
    hip_center_x = (hip_deck_start + hip_deck_end) / 2.0
    hip_deck_length = hip_deck_end - hip_deck_start
    hip_mattress = _rounded_cushion_mesh(
        name="hip_mattress",
        center_x=hip_center_x,
        length=hip_deck_length - 0.04,
        width=0.82,
        z_min=deck_top_z - 0.030,
        z_max=deck_top_z + 0.075,
        edge_taper=0.045,
        softness=0.022,
    )

    # Thigh mattress in thigh local frame (full length, meets both hinge lines).
    thigh_deck_length = thigh_length
    thigh_deck_center_local = thigh_length / 2.0
    thigh_mattress = _rounded_cushion_mesh(
        name="thigh_mattress",
        center_x=thigh_deck_center_local,
        length=thigh_deck_length - 0.04,
        width=0.82,
        z_min=-0.002,
        z_max=0.075,
        edge_taper=0.040,
        softness=0.022,
    )

    # Calf mattress in calf local frame (starts at proximal hinge, no inset).
    calf_deck_length = calf_length
    calf_deck_center_local = calf_length / 2.0
    calf_mattress = _rounded_cushion_mesh(
        name="calf_mattress",
        center_x=calf_deck_center_local,
        length=calf_deck_length - 0.04,
        width=0.82,
        z_min=-0.002,
        z_max=0.075,
        edge_taper=0.040,
        softness=0.022,
    )

    # ============================================================
    # BASE FRAME (root)
    # ============================================================
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

    # Cross-rails at structural positions (all below deck panels).
    for x, suffix in [
        (-0.96, "head"),
        (backrest_hinge_x, "backrest_hinge"),
        (thigh_hinge_x, "thigh_hinge"),
        (0.96, "foot"),
    ]:
        _cylinder_y(
            base,
            name=f"{suffix}_cross_rail",
            x=x,
            y=0.0,
            z=0.558,
            length=0.94,
            radius=tube_r,
            material=WHITE_STEEL,
        )

    # Fixed hip deck panel and hip mattress.
    base.visual(
        Box((hip_deck_length, 0.86, 0.030)),
        origin=_origin(hip_center_x, 0.0, deck_top_z - 0.015),
        material=OFF_WHITE,
        name="hip_deck",
    )
    base.visual(hip_mattress, origin=Origin(), material=BLUE_FABRIC, name="hip_mattress")

    # Low tubular head and foot boards.
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

    # Four swivel casters.
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

    # Hinge barrels at the backrest division.
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

    # Hinge barrels at the thigh section division.
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            base,
            name=f"thigh_hinge_barrel_{suffix}",
            x=thigh_hinge_x,
            y=y,
            z=deck_top_z,
            length=0.12,
            radius=0.018,
            material=WHITE_STEEL,
        )

    # ============================================================
    # BACKREST (unchanged from parent baseline)
    # ============================================================
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

    # ============================================================
    # THIGH SECTION
    # ============================================================
    thigh = model.part("thigh_section")

    # Deck panel extends in +X from the thigh hinge (inset from both hinge lines).
    thigh.visual(
        Box((thigh_deck_length, 0.86, 0.030)),
        origin=_origin(thigh_deck_center_local, 0.0, -0.015),
        material=OFF_WHITE,
        name="thigh_deck",
    )
    thigh.visual(thigh_mattress, origin=Origin(), material=BLUE_FABRIC, name="thigh_mattress")

    # Proximal hinge tube at the thigh hinge line (local origin).
    _cylinder_y(
        thigh,
        name="thigh_proximal_hinge_tube",
        x=0.0,
        y=0.0,
        z=0.0,
        length=0.44,
        radius=0.018,
        material=WHITE_STEEL,
    )
    for y, suffix in [(-0.365, "0"), (0.365, "1")]:
        _cylinder_y(
            thigh,
            name=f"thigh_proximal_tube_{suffix}",
            x=0.0,
            y=y,
            z=0.0,
            length=0.13,
            radius=0.018,
            material=WHITE_STEEL,
        )
    for y, suffix in [(-0.34, "0"), (0.34, "1")]:
        thigh.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(0.035, y, -0.006),
            material=WHITE_STEEL,
            name=f"thigh_proximal_leaf_{suffix}",
        )

    # Distal hinge barrels for the calf section (at the knee end).
    for y, suffix in [(-0.49, "0"), (0.49, "1")]:
        _cylinder_y(
            thigh,
            name=f"calf_hinge_barrel_{suffix}",
            x=thigh_length,
            y=y,
            z=0.0,
            length=0.12,
            radius=0.018,
            material=WHITE_STEEL,
        )

    # ============================================================
    # CALF SECTION
    # ============================================================
    calf = model.part("calf_section")

    # Deck panel extends in +X from the calf (knee) hinge (inset from proximal hinge).
    calf.visual(
        Box((calf_deck_length, 0.86, 0.030)),
        origin=_origin(calf_deck_center_local, 0.0, -0.015),
        material=OFF_WHITE,
        name="calf_deck",
    )
    calf.visual(calf_mattress, origin=Origin(), material=BLUE_FABRIC, name="calf_mattress")

    # Proximal hinge tube at the knee hinge line (local origin).
    _cylinder_y(
        calf,
        name="calf_proximal_hinge_tube",
        x=0.0,
        y=0.0,
        z=0.0,
        length=0.44,
        radius=0.018,
        material=WHITE_STEEL,
    )
    for y, suffix in [(-0.365, "0"), (0.365, "1")]:
        _cylinder_y(
            calf,
            name=f"calf_proximal_tube_{suffix}",
            x=0.0,
            y=y,
            z=0.0,
            length=0.13,
            radius=0.018,
            material=WHITE_STEEL,
        )
    for y, suffix in [(-0.34, "0"), (0.34, "1")]:
        calf.visual(
            Box((0.070, 0.045, 0.018)),
            origin=_origin(0.035, y, -0.006),
            material=WHITE_STEEL,
            name=f"calf_proximal_leaf_{suffix}",
        )

    # ============================================================
    # ARTICULATIONS
    # ============================================================

    # Backrest: positive q raises the head end upward.
    model.articulation(
        "base_to_backrest",
        ArticulationType.REVOLUTE,
        parent=base,
        child=backrest,
        origin=_origin(backrest_hinge_x, 0.0, deck_top_z),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.15),
    )

    # Thigh section: axis (0,-1,0) so positive q lifts the knee end upward.
    model.articulation(
        "base_to_thigh_section",
        ArticulationType.REVOLUTE,
        parent=base,
        child=thigh,
        origin=_origin(thigh_hinge_x, 0.0, deck_top_z),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=0.70),
    )

    # Calf section: axis (0,1,0) so positive q drops the foot end downward
    # from the raised thigh, creating the raised-knee profiling contour.
    model.articulation(
        "thigh_section_to_calf_section",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=calf,
        origin=_origin(calf_hinge_x_local, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.30, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    thigh = object_model.get_part("thigh_section")
    calf = object_model.get_part("calf_section")
    backrest_hinge = object_model.get_articulation("base_to_backrest")
    thigh_hinge = object_model.get_articulation("base_to_thigh_section")
    calf_hinge = object_model.get_articulation("thigh_section_to_calf_section")

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

    # --- Backrest checks ---
    ctx.expect_overlap(
        backrest,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="back_deck",
        elem_b="hip_deck",
        name="backrest and hip deck share bed width",
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

    # --- Thigh section checks (TARGET-specific) ---
    ctx.expect_overlap(
        thigh,
        base,
        axes="y",
        min_overlap=0.70,
        elem_a="thigh_deck",
        elem_b="hip_deck",
        name="thigh_section and hip deck share bed width",
    )

    # Thigh section lifts the knee end upward with positive q.
    flat_knee = ctx.part_element_world_aabb(thigh, elem="thigh_deck")
    with ctx.pose({thigh_hinge: 0.55}):
        raised_knee = ctx.part_element_world_aabb(thigh, elem="thigh_deck")
        ctx.check(
            "base_to_thigh_section lifts knee end upward",
            flat_knee is not None
            and raised_knee is not None
            and raised_knee[1][2] > flat_knee[1][2] + 0.10,
            details=f"flat={flat_knee}, raised={raised_knee}",
        )

    # --- Calf section checks (TARGET-specific) ---
    ctx.expect_overlap(
        calf,
        thigh,
        axes="y",
        min_overlap=0.70,
        elem_a="calf_deck",
        elem_b="thigh_deck",
        name="calf_section and thigh_section share bed width",
    )

    # Calf section drops the foot end with positive q (creating knee contour).
    flat_foot = ctx.part_element_world_aabb(calf, elem="calf_deck")
    with ctx.pose({calf_hinge: 0.60}):
        dropped_foot = ctx.part_element_world_aabb(calf, elem="calf_deck")
        ctx.check(
            "thigh_section_to_calf_section drops foot end downward",
            flat_foot is not None
            and dropped_foot is not None
            and dropped_foot[0][2] < flat_foot[0][2] - 0.05,
            details=f"flat={flat_foot}, dropped={dropped_foot}",
        )

    # Full profiling pose: thigh up + calf drops foot below knee contour.
    with ctx.pose({thigh_hinge: 0.55, calf_hinge: 0.70}):
        knee_pos = ctx.part_element_world_aabb(thigh, elem="thigh_deck")
        foot_pos = ctx.part_element_world_aabb(calf, elem="calf_deck")
        # Knee (thigh max_z) should be well above the calf's lowest point (foot end min_z).
        ctx.check(
            "profiled bed forms raised-knee contour",
            knee_pos is not None
            and foot_pos is not None
            and knee_pos[1][2] > foot_pos[0][2] + 0.05,
            details=f"knee={knee_pos}, foot={foot_pos}",
        )

    # --- Intentional hinge-line overlap allowances ---
    # The calf hinge tubes sit at the knee hinge line and slightly intrude
    # into the thigh deck panel edge — a realistic hinge-barrel adjacency.
    for tube_name in ("calf_proximal_hinge_tube", "calf_proximal_tube_0", "calf_proximal_tube_1"):
        ctx.allow_overlap(
            calf,
            thigh,
            elem_a=tube_name,
            elem_b="thigh_deck",
            reason=f"{tube_name} at the knee hinge line slightly overlaps the thigh deck edge, as in a real hinge-barrel joint.",
        )
    ctx.expect_gap(
        calf,
        thigh,
        axis="x",
        max_penetration=0.020,
        positive_elem="calf_deck",
        negative_elem="thigh_deck",
        name="calf deck meets thigh deck at knee hinge with minimal penetration",
    )

    return ctx.report()


object_model = build_object_model()
