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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _safe_fillet(shape, selector: str, radius: float):
    try:
        return shape.edges(selector).fillet(radius)
    except Exception:
        return shape


def _tube_x(outer_radius: float, inner_radius: float, length: float):
    """CadQuery tube with its axis along local +X, centered at the origin."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _cylinder_x(radius: float, length: float):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _triangular_rib(points_xz: list[tuple[float, float]], width_y: float):
    return (
        cq.Workplane("XZ")
        .polyline(points_xz)
        .close()
        .extrude(width_y)
        .translate((0.0, -width_y / 2.0, 0.0))
    )


def _base_plate_shape():
    plate = cq.Workplane("XY").box(0.30, 0.15, 0.024)
    for x in (-0.14, 0.14):
        for y in (-0.075, 0.075):
            ear = cq.Workplane("XY").center(x, y).cylinder(0.024, 0.036)
            plate = plate.union(ear)
    for x in (-0.14, 0.14):
        for y in (-0.075, 0.075):
            hole = cq.Workplane("XY").center(x, y).cylinder(0.070, 0.012)
            plate = plate.cut(hole)
    return _safe_fillet(plate, "|Z", 0.006)


def _serrated_bar_set(name_side: str):
    # A single mesh made from several raised bars so each jaw plate reads as a
    # replaceable serrated steel insert rather than a flat gray block.
    bars = cq.Workplane("XY")
    for i, z in enumerate((-0.015, -0.010, -0.005, 0.000, 0.005, 0.010, 0.015)):
        bar = (
            cq.Workplane("XY")
            .box(0.003, 0.100, 0.0018)
            .translate((0.0, 0.0, z))
            .rotate((0, 0, 0), (1, 0, 0), 0.0)
        )
        if i == 0:
            bars = bar
        else:
            bars = bars.union(bar)
    return bars


def _lock_wing_shape():
    stem_disk = _cylinder_x(0.012, 0.012)
    wing_a = cq.Workplane("XY").box(0.010, 0.055, 0.010)
    wing_b = cq.Workplane("XY").box(0.010, 0.018, 0.030)
    return stem_disk.union(wing_a).union(wing_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="red_bench_vise",
        meta={
            "category": "Industrial / Industrial vice",
            "bolt_hole_count": 4,
            "axis": "X",
            "lead_screw_note": "The screw/handle spins on the moving jaw axis; positive prismatic travel closes the jaw gap.",
        },
    )

    rough_red = model.material("rough_chipped_red", rgba=(0.82, 0.03, 0.08, 1.0))
    dark_red = model.material("dark_red_recesses", rgba=(0.45, 0.02, 0.03, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    jaw_steel = model.material("textured_gray_jaw_steel", rgba=(0.43, 0.43, 0.40, 1.0))
    black = model.material("blackened_screw_heads", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root bench mounting base: broad cast plate with rounded ears, true bolt
    # holes, and the lower half of the swivel turntable.
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate_bolt_holes"),
        origin=Origin(xyz=(0.055, 0.0, 0.012)),
        material=rough_red,
        name="base_plate_bolt_holes",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.038),
        origin=Origin(xyz=(0.055, 0.0, 0.043)),
        material=rough_red,
        name="lower_swivel_cylinder",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.004),
        origin=Origin(xyz=(0.055, 0.0, 0.064)),
        material=dark_red,
        name="swivel_seam_ring",
    )

    # Swiveling vise body: fixed rear jaw, rail guides, anvil pad, screw guide,
    # cast ribs, and side lock screw detail all rotate together above the base.
    body = model.part("swivel_body")
    body.visual(
        Cylinder(radius=0.088, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=rough_red,
        name="upper_swivel_disk",
    )
    for idx, y in enumerate((-0.045, 0.045)):
        body.visual(
            Box((0.210, 0.030, 0.050)),
            origin=Origin(xyz=(0.035, y, 0.045)),
            material=rough_red,
            name=f"cast_saddle_{idx}",
        )
    body.visual(
        Box((0.205, 0.022, 0.044)),
        origin=Origin(xyz=(0.030, 0.050, 0.086)),
        material=rough_red,
        name="rail_guide_0",
    )
    body.visual(
        Box((0.205, 0.022, 0.044)),
        origin=Origin(xyz=(0.030, -0.050, 0.086)),
        material=rough_red,
        name="rail_guide_1",
    )
    body.visual(
        mesh_from_cadquery(_tube_x(0.034, 0.017, 0.085), "body_screw_tube"),
        origin=Origin(xyz=(-0.018, 0.0, 0.035)),
        material=rough_red,
        name="body_screw_tube",
    )
    body.visual(
        Box((0.074, 0.140, 0.155)),
        origin=Origin(xyz=(0.083, 0.0, 0.180)),
        material=rough_red,
        name="fixed_jaw_casting",
    )
    body.visual(
        Box((0.150, 0.098, 0.060)),
        origin=Origin(xyz=(0.165, 0.0, 0.140)),
        material=rough_red,
        name="rear_tail_beam",
    )
    body.visual(
        Box((0.085, 0.118, 0.020)),
        origin=Origin(xyz=(0.119, 0.0, 0.245)),
        material=steel,
        name="rear_anvil_pad",
    )
    body.visual(
        Box((0.012, 0.118, 0.045)),
        origin=Origin(xyz=(0.040, 0.0, 0.175)),
        material=jaw_steel,
        name="fixed_jaw_plate",
    )
    body.visual(
        mesh_from_cadquery(_serrated_bar_set("fixed"), "fixed_serrations"),
        origin=Origin(xyz=(0.034, 0.0, 0.175)),
        material=steel,
        name="fixed_serrations",
    )
    for idx, y in enumerate((-0.036, 0.036)):
        body.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.032, y, 0.175), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"fixed_plate_screw_{idx}",
        )
    body.visual(
        mesh_from_cadquery(
            _triangular_rib([(-0.075, 0.020), (0.072, 0.020), (0.072, 0.130)], 0.022),
            "cast_rib_0",
        ),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=rough_red,
        name="cast_rib_0",
    )
    body.visual(
        mesh_from_cadquery(
            _triangular_rib([(-0.075, 0.020), (0.072, 0.020), (0.072, 0.130)], 0.022),
            "cast_rib_1",
        ),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=rough_red,
        name="cast_rib_1",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.045, -0.076, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rough_red,
        name="side_lock_boss",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.045, -0.112, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_lock_stem",
    )
    body.visual(
        mesh_from_cadquery(_lock_wing_shape(), "side_lock_wing"),
        origin=Origin(xyz=(0.045, -0.150, 0.035), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="side_lock_wing",
    )

    # Moving jaw and sliding rectangular rail.  At q=0 it is open; positive
    # prismatic travel moves along +X toward the fixed jaw plate.
    sliding = model.part("sliding_jaw")
    sliding.visual(
        Box((0.300, 0.050, 0.035)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=rough_red,
        name="sliding_rectangular_rail",
    )
    sliding.visual(
        Box((0.064, 0.132, 0.146)),
        origin=Origin(xyz=(-0.016, 0.0, 0.052)),
        material=rough_red,
        name="moving_jaw_casting",
    )
    sliding.visual(
        mesh_from_cadquery(_tube_x(0.037, 0.017, 0.040), "front_screw_boss"),
        origin=Origin(xyz=(-0.078, 0.0, -0.050)),
        material=rough_red,
        name="front_screw_boss",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        sliding.visual(
            Box((0.060, 0.018, 0.050)),
            origin=Origin(xyz=(-0.075, y, 0.005)),
            material=rough_red,
            name=f"boss_bridge_{idx}",
        )
    sliding.visual(
        Box((0.012, 0.118, 0.045)),
        origin=Origin(xyz=(0.022, 0.0, 0.090)),
        material=jaw_steel,
        name="moving_jaw_plate",
    )
    sliding.visual(
        mesh_from_cadquery(_serrated_bar_set("moving"), "moving_serrations"),
        origin=Origin(xyz=(0.029, 0.0, 0.090)),
        material=steel,
        name="moving_serrations",
    )
    for idx, y in enumerate((-0.036, 0.036)):
        sliding.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.031, y, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"moving_plate_screw_{idx}",
        )

    # Lead screw and T-handle spin as a child of the moving jaw, so the entire
    # handle assembly remains captured by the front screw boss while sliding.
    screw = model.part("screw_handle")
    screw.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lead_screw",
    )
    for idx, x in enumerate((0.035, 0.070, 0.105, 0.140, 0.175, 0.210, 0.245, 0.280)):
        screw.visual(
            Cylinder(radius=0.0113, length=0.0035),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"thread_ridge_{idx}",
        )
    screw.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(-0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_screw_flange",
    )
    screw.visual(
        Cylinder(radius=0.008, length=0.245),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="t_handle",
    )
    for idx, y in enumerate((-0.132, 0.132)):
        screw.visual(
            Cylinder(radius=0.014, length=0.024),
            origin=Origin(xyz=(-0.060, y * 0.93, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"handle_cap_{idx}",
        )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.055, 0.0, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.70, effort=80.0, velocity=0.5),
    )
    model.articulation(
        "body_to_sliding_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=sliding,
        origin=Origin(xyz=(-0.130, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.100, effort=500.0, velocity=0.05),
    )
    model.articulation(
        "sliding_jaw_to_screw",
        ArticulationType.CONTINUOUS,
        parent=sliding,
        child=screw,
        origin=Origin(xyz=(-0.078, 0.0, -0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("swivel_body")
    sliding = object_model.get_part("sliding_jaw")
    screw = object_model.get_part("screw_handle")
    slide_joint = object_model.get_articulation("body_to_sliding_jaw")
    screw_joint = object_model.get_articulation("sliding_jaw_to_screw")

    def visual_names(part):
        return {v.name for v in part.visuals}

    all_aabbs = [ctx.part_world_aabb(p) for p in (base, body, sliding, screw)]
    mins = [min(aabb[0][i] for aabb in all_aabbs if aabb) for i in range(3)]
    maxs = [max(aabb[1][i] for aabb in all_aabbs if aabb) for i in range(3)]
    dims = [maxs[i] - mins[i] for i in range(3)]
    ctx.check(
        "bench vise silhouette is long, wide, and tall",
        dims[0] > 0.48 and dims[1] > 0.24 and dims[2] > 0.23,
        details=f"overall dims={dims}",
    )
    ctx.check(
        "base has four bench bolt holes",
        object_model.meta.get("bolt_hole_count") == 4
        and "base_plate_bolt_holes" in visual_names(base),
        details=f"meta={object_model.meta}, visuals={visual_names(base)}",
    )
    ctx.check(
        "fixed and moving jaws are present",
        {"fixed_jaw_casting", "fixed_jaw_plate"}.issubset(visual_names(body))
        and {"moving_jaw_casting", "moving_jaw_plate"}.issubset(visual_names(sliding)),
    )
    ctx.expect_overlap(
        body,
        sliding,
        axes="yz",
        elem_a="fixed_jaw_plate",
        elem_b="moving_jaw_plate",
        min_overlap=0.035,
        name="jaw plates are parallel and vertically aligned",
    )
    ctx.expect_gap(
        body,
        sliding,
        axis="x",
        positive_elem="fixed_jaw_plate",
        negative_elem="moving_jaw_plate",
        min_gap=0.080,
        max_gap=0.160,
        name="open jaw gap faces along vise axis",
    )
    ctx.check(
        "serrated replaceable plates and black screws are modeled",
        {"fixed_serrations", "fixed_plate_screw_0", "fixed_plate_screw_1"}.issubset(
            visual_names(body)
        )
        and {"moving_serrations", "moving_plate_screw_0", "moving_plate_screw_1"}.issubset(
            visual_names(sliding)
        ),
    )
    ctx.check(
        "lead screw and T handle are modeled",
        {"lead_screw", "front_screw_flange", "t_handle", "handle_cap_0", "handle_cap_1"}.issubset(
            visual_names(screw)
        ),
    )
    ctx.check(
        "side lock screw detail is modeled",
        {"side_lock_boss", "side_lock_stem", "side_lock_wing"}.issubset(visual_names(body)),
    )

    rest_pos = ctx.part_world_position(sliding)
    with ctx.pose({slide_joint: 0.100}):
        closed_pos = ctx.part_world_position(sliding)
        ctx.expect_gap(
            body,
            sliding,
            axis="x",
            positive_elem="fixed_jaw_plate",
            negative_elem="moving_jaw_plate",
            min_gap=0.0,
            max_gap=0.070,
            name="moving jaw closes toward fixed jaw",
        )
    ctx.check(
        "non-fixed jaw has positive closing travel",
        rest_pos is not None
        and closed_pos is not None
        and closed_pos[0] > rest_pos[0] + 0.080,
        details=f"rest={rest_pos}, closed={closed_pos}",
    )

    handle_aabb_0 = ctx.part_element_world_aabb(screw, elem="t_handle")
    with ctx.pose({screw_joint: math.pi / 2.0}):
        handle_aabb_90 = ctx.part_element_world_aabb(screw, elem="t_handle")
    if handle_aabb_0 and handle_aabb_90:
        dy_0 = handle_aabb_0[1][1] - handle_aabb_0[0][1]
        dz_0 = handle_aabb_0[1][2] - handle_aabb_0[0][2]
        dy_90 = handle_aabb_90[1][1] - handle_aabb_90[0][1]
        dz_90 = handle_aabb_90[1][2] - handle_aabb_90[0][2]
        spin_ok = dy_0 > dz_0 * 3.0 and dz_90 > dy_90 * 3.0
    else:
        spin_ok = False
        dy_0 = dz_0 = dy_90 = dz_90 = None
    ctx.check(
        "lead screw joint visibly spins T handle",
        spin_ok,
        details=f"dy0={dy_0}, dz0={dz_0}, dy90={dy_90}, dz90={dz_90}",
    )

    return ctx.report()


object_model = build_object_model()
