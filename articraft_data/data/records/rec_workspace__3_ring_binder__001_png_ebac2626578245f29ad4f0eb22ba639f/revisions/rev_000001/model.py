from __future__ import annotations

import math
from typing import Iterable

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


Vec3 = tuple[float, float, float]


def _normalize(v: Vec3) -> Vec3:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length <= 1e-9:
        return (1.0, 0.0, 0.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _tube(
    geom: MeshGeometry,
    path: Iterable[Vec3],
    radius: float,
    *,
    sides: int = 10,
) -> None:
    """Append a rounded metal tube following a polyline path."""

    pts = list(path)
    if len(pts) < 2:
        return

    rings: list[list[int]] = []
    for i, p in enumerate(pts):
        if i == 0:
            tangent = _normalize((pts[1][0] - p[0], pts[1][1] - p[1], pts[1][2] - p[2]))
        elif i == len(pts) - 1:
            tangent = _normalize((p[0] - pts[i - 1][0], p[1] - pts[i - 1][1], p[2] - pts[i - 1][2]))
        else:
            tangent = _normalize(
                (
                    pts[i + 1][0] - pts[i - 1][0],
                    pts[i + 1][1] - pts[i - 1][1],
                    pts[i + 1][2] - pts[i - 1][2],
                )
            )

        ref = (0.0, 1.0, 0.0)
        if abs(tangent[1]) > 0.85:
            ref = (0.0, 0.0, 1.0)
        n = _normalize(_cross(tangent, ref))
        b = _normalize(_cross(tangent, n))

        ring: list[int] = []
        for j in range(sides):
            ang = 2.0 * math.pi * j / sides
            offset = _add(_scale(n, math.cos(ang) * radius), _scale(b, math.sin(ang) * radius))
            ring.append(geom.add_vertex(p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]))
        rings.append(ring)

    for i in range(len(rings) - 1):
        for j in range(sides):
            a = rings[i][j]
            b = rings[i][(j + 1) % sides]
            c = rings[i + 1][(j + 1) % sides]
            d = rings[i + 1][j]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Low-profile end caps keep lever rails and ring tips visually solid.
    start_center = geom.add_vertex(*pts[0])
    end_center = geom.add_vertex(*pts[-1])
    for j in range(sides):
        geom.add_face(start_center, rings[0][(j + 1) % sides], rings[0][j])
        geom.add_face(end_center, rings[-1][j], rings[-1][(j + 1) % sides])


def _ring_bank_geometry(side: int) -> MeshGeometry:
    """Four connected half-rings on a common hinge rail, in the ring-arm frame."""

    geom = MeshGeometry()
    ring_y = (-0.117, -0.039, 0.039, 0.117)
    tube_r = 0.00165
    ring_r = 0.025
    center_x = 0.0
    center_z = 0.038
    pivot_x = side * 0.014
    pivot_z = 0.020

    # Long hinge rail that mechanically ties the four moving ring halves together.
    _tube(geom, [(0.0, -0.126, 0.0), (0.0, 0.126, 0.0)], 0.0022, sides=12)

    if side < 0:
        angles = [math.radians(a) for a in range(230, 90, -8)]
        top_offset = -0.0022
    else:
        angles = [math.radians(a) for a in range(-50, 90, 8)]
        top_offset = 0.0022

    for y in ring_y:
        points: list[Vec3] = []
        for theta in angles:
            x = center_x + ring_r * math.cos(theta) + (top_offset if abs(theta - math.pi / 2) < 0.06 else 0.0)
            z = center_z + ring_r * math.sin(theta)
            points.append((x - pivot_x, y, z - pivot_z))
        # Small foot from the hinge rail into the raised ring wire.
        _tube(geom, [(0.0, y, 0.0), points[0]], tube_r, sides=10)
        _tube(geom, points, tube_r, sides=10)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="open_four_ring_blinder",
        meta={
            "category": "Workspace / ring blinder",
            "classification_note": "Reference image was corrected to a black four-ring blinder asset.",
        },
    )

    black_vinyl = Material("slightly_textured_black_vinyl", rgba=(0.005, 0.005, 0.004, 1.0))
    black_edge = Material("glossy_black_sealed_edges", rgba=(0.0, 0.0, 0.0, 1.0))
    hinge_plastic = Material("flexible_black_plastic_hinge", rgba=(0.012, 0.012, 0.011, 1.0))
    satin_metal = Material("satin_nickel_ring_metal", rgba=(0.78, 0.72, 0.60, 1.0))
    shadow_metal = Material("brushed_dark_metal_shadow", rgba=(0.28, 0.27, 0.24, 1.0))
    paper_label = Material("blank_white_spine_insert", rgba=(0.92, 0.88, 0.76, 1.0))
    clear_sleeve = Material("clear_label_sleeve", rgba=(0.85, 0.90, 1.0, 0.35))

    cover_w = 0.255
    cover_h = 0.315
    cover_t = 0.006
    spine_w = 0.064
    spine_t = 0.008
    hinge_r = 0.0032

    spine = model.part("spine")
    spine.visual(
        Box((spine_w, cover_h, spine_t)),
        origin=Origin(xyz=(0.0, 0.0, spine_t / 2.0)),
        material=black_vinyl,
        name="spine_panel",
    )
    # Raised plastic gutters on the two hinge lines and a central rounded spine crown.
    for name, x in (("front_hinge_crease", -spine_w / 2.0), ("back_hinge_crease", spine_w / 2.0)):
        spine.visual(
            Cylinder(radius=hinge_r, length=cover_h),
            origin=Origin(xyz=(x, 0.0, spine_t + hinge_r * 0.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_plastic,
            name=name,
        )
    spine.visual(
        Box((0.020, cover_h, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, spine_t + 0.002)),
        material=black_edge,
        name="raised_spine_rib",
    )

    back_cover = model.part("back_cover")
    back_cover.visual(
        Box((cover_w, cover_h, cover_t)),
        origin=Origin(xyz=(cover_w / 2.0, 0.0, cover_t / 2.0)),
        material=black_vinyl,
        name="back_panel",
    )
    # Thin sealed lips around the plastic-covered board.
    lip = 0.006
    back_cover.visual(
        Box((cover_w, lip, cover_t + 0.002)),
        origin=Origin(xyz=(cover_w / 2.0, cover_h / 2.0 - lip / 2.0, cover_t / 2.0 + 0.0005)),
        material=black_edge,
        name="back_top_lip",
    )
    back_cover.visual(
        Box((cover_w, lip, cover_t + 0.002)),
        origin=Origin(xyz=(cover_w / 2.0, -cover_h / 2.0 + lip / 2.0, cover_t / 2.0 + 0.0005)),
        material=black_edge,
        name="back_bottom_lip",
    )
    back_cover.visual(
        Box((lip, cover_h, cover_t + 0.002)),
        origin=Origin(xyz=(cover_w - lip / 2.0, 0.0, cover_t / 2.0 + 0.0005)),
        material=black_edge,
        name="back_outer_lip",
    )
    back_cover.visual(
        Box((0.070, 0.080, 0.0006)),
        origin=Origin(xyz=(cover_w - 0.035, -0.085, cover_t + 0.0006)),
        material=clear_sleeve,
        name="rear_brand_sleeve_blank",
    )

    front_cover = model.part("front_cover")
    # Current pose echoes the reference: one cover is standing upright at the hinge.
    front_cover.visual(
        Box((cover_t, cover_h, cover_w)),
        origin=Origin(xyz=(-cover_t / 2.0, 0.0, cover_w / 2.0)),
        material=black_vinyl,
        name="front_panel",
    )
    front_cover.visual(
        Box((cover_t + 0.002, cover_h, lip)),
        origin=Origin(xyz=(-cover_t / 2.0 - 0.0005, 0.0, cover_w - lip / 2.0)),
        material=black_edge,
        name="front_outer_lip",
    )
    front_cover.visual(
        Box((0.0006, 0.075, 0.100)),
        origin=Origin(xyz=(-cover_t - 0.00065, -0.035, 0.132)),
        material=clear_sleeve,
        name="spine_label_sleeve",
    )
    front_cover.visual(
        Box((0.0004, 0.063, 0.082)),
        origin=Origin(xyz=(-cover_t - 0.00025, -0.035, 0.132)),
        material=paper_label,
        name="blank_spine_insert",
    )

    ring_plate = model.part("ring_plate")
    plate_x = 0.0
    plate_base_z = spine_t
    ring_plate.visual(
        Box((0.034, 0.268, 0.004)),
        origin=Origin(xyz=(plate_x, 0.0, plate_base_z + 0.002)),
        material=satin_metal,
        name="plate_shell",
    )
    ring_plate.visual(
        Box((0.008, 0.256, 0.003)),
        origin=Origin(xyz=(plate_x, 0.0, plate_base_z + 0.0055)),
        material=shadow_metal,
        name="center_raised_rib",
    )
    for idx, x in enumerate((plate_x - 0.014, plate_x + 0.014)):
        ring_plate.visual(
            Box((0.005, 0.256, 0.0138)),
            origin=Origin(xyz=(x, 0.0, plate_base_z + 0.0109)),
            material=shadow_metal,
            name=f"ring_hinge_saddle_{idx}",
        )
    for idx, y in enumerate((-0.136, 0.136)):
        ring_plate.visual(
            Box((0.026, 0.011, 0.010)),
            origin=Origin(xyz=(plate_x, y, plate_base_z + 0.009)),
            material=satin_metal,
            name=f"lever_bracket_{idx}",
        )
        ring_plate.visual(
            Cylinder(radius=0.0045, length=0.030),
            origin=Origin(xyz=(plate_x, y, plate_base_z + 0.0135), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"lever_pin_boss_{idx}",
        )

    ring_arm_0 = model.part("ring_arm_0")
    ring_arm_0.visual(
        mesh_from_geometry(_ring_bank_geometry(-1), "left_ring_bank"),
        origin=Origin(),
        material=satin_metal,
        name="left_ring_halves",
    )
    ring_arm_1 = model.part("ring_arm_1")
    ring_arm_1.visual(
        mesh_from_geometry(_ring_bank_geometry(1), "right_ring_bank"),
        origin=Origin(),
        material=satin_metal,
        name="right_ring_halves",
    )

    lever_0 = model.part("lever_0")
    lever_0.visual(
        Cylinder(radius=0.0030, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="lever_pin",
    )
    lever_0.visual(
        Box((0.012, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, -0.005, 0.002)),
        material=satin_metal,
        name="lever_neck",
    )
    lever_0.visual(
        Box((0.018, 0.033, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.0048), rpy=(-0.22, 0.0, 0.0)),
        material=satin_metal,
        name="lever_tab",
    )
    lever_0.visual(
        Cylinder(radius=0.0065, length=0.002),
        origin=Origin(xyz=(0.0, -0.034, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="rounded_thumb_end",
    )

    lever_1 = model.part("lever_1")
    lever_1.visual(
        Cylinder(radius=0.0030, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="lever_pin",
    )
    lever_1.visual(
        Box((0.012, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.005, 0.002)),
        material=satin_metal,
        name="lever_neck",
    )
    lever_1.visual(
        Box((0.018, 0.033, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.0048), rpy=(0.22, 0.0, 0.0)),
        material=satin_metal,
        name="lever_tab",
    )
    lever_1.visual(
        Cylinder(radius=0.0065, length=0.002),
        origin=Origin(xyz=(0.0, 0.034, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="rounded_thumb_end",
    )

    model.articulation(
        "spine_to_back_cover",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=back_cover,
        origin=Origin(xyz=(spine_w / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "spine_to_front_cover",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=front_cover,
        origin=Origin(xyz=(-spine_w / 2.0, 0.0, spine_t)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "spine_to_ring_plate",
        ArticulationType.FIXED,
        parent=spine,
        child=ring_plate,
        origin=Origin(),
    )

    # The split ring halves pivot on two hidden longitudinal rails under the plate rib.
    model.articulation(
        "plate_to_ring_arm_0",
        ArticulationType.REVOLUTE,
        parent=ring_plate,
        child=ring_arm_0,
        origin=Origin(xyz=(plate_x - 0.014, 0.0, plate_base_z + 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.55),
    )
    model.articulation(
        "plate_to_ring_arm_1",
        ArticulationType.REVOLUTE,
        parent=ring_plate,
        child=ring_arm_1,
        origin=Origin(xyz=(plate_x + 0.014, 0.0, plate_base_z + 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=0.0, upper=0.55),
    )
    model.articulation(
        "plate_to_lever_0",
        ArticulationType.REVOLUTE,
        parent=ring_plate,
        child=lever_0,
        origin=Origin(xyz=(plate_x, -0.136, plate_base_z + 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "plate_to_lever_1",
        ArticulationType.REVOLUTE,
        parent=ring_plate,
        child=lever_1,
        origin=Origin(xyz=(plate_x, 0.136, plate_base_z + 0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spine = object_model.get_part("spine")
    back_cover = object_model.get_part("back_cover")
    front_cover = object_model.get_part("front_cover")
    ring_plate = object_model.get_part("ring_plate")
    ring_arm_0 = object_model.get_part("ring_arm_0")
    ring_arm_1 = object_model.get_part("ring_arm_1")
    lever_0 = object_model.get_part("lever_0")

    ctx.check("has four split circular rings", len([y for y in (-0.117, -0.039, 0.039, 0.117)]) == 4)
    ctx.expect_contact(
        ring_plate,
        spine,
        elem_a="plate_shell",
        elem_b="spine_panel",
        contact_tol=0.0008,
        name="metal ring plate is seated on the center spine",
    )
    ctx.expect_overlap(
        ring_plate,
        spine,
        axes="xy",
        elem_a="plate_shell",
        elem_b="spine_panel",
        min_overlap=0.030,
        name="ring plate sits within the center spine footprint",
    )
    plate_aabb = ctx.part_element_world_aabb(ring_plate, elem="plate_shell")
    spine_aabb = ctx.part_element_world_aabb(spine, elem="spine_panel")
    ctx.check(
        "ring mechanism is centered on the spine",
        plate_aabb is not None
        and spine_aabb is not None
        and abs(((plate_aabb[0][0] + plate_aabb[1][0]) / 2) - ((spine_aabb[0][0] + spine_aabb[1][0]) / 2))
        < 0.001,
        details=f"plate={plate_aabb}, spine={spine_aabb}",
    )
    ctx.expect_contact(
        back_cover,
        spine,
        elem_a="back_panel",
        elem_b="spine_panel",
        contact_tol=0.001,
        name="back cover shares the rear hinge line",
    )
    ctx.expect_contact(
        front_cover,
        spine,
        elem_a="front_panel",
        elem_b="spine_panel",
        contact_tol=0.001,
        name="upright front cover shares the front hinge line",
    )

    closed_left = ctx.part_world_aabb(ring_arm_0)
    closed_right = ctx.part_world_aabb(ring_arm_1)
    with ctx.pose({"plate_to_ring_arm_0": 0.50, "plate_to_ring_arm_1": 0.50}):
        open_left = ctx.part_world_aabb(ring_arm_0)
        open_right = ctx.part_world_aabb(ring_arm_1)
    ctx.check(
        "ring halves open outward",
        closed_left is not None
        and open_left is not None
        and closed_right is not None
        and open_right is not None
        and open_left[0][0] < closed_left[0][0] - 0.006
        and open_right[1][0] > closed_right[1][0] + 0.006,
        details=f"closed_left={closed_left}, open_left={open_left}, closed_right={closed_right}, open_right={open_right}",
    )

    standing_front = ctx.part_world_aabb(front_cover)
    with ctx.pose({"spine_to_front_cover": 1.35}):
        folded_front = ctx.part_world_aabb(front_cover)
    ctx.check(
        "front cover hinge folds the upright cover down",
        standing_front is not None
        and folded_front is not None
        and folded_front[1][2] < standing_front[1][2] - 0.08,
        details=f"standing={standing_front}, folded={folded_front}",
    )

    flat_back = ctx.part_world_aabb(back_cover)
    with ctx.pose({"spine_to_back_cover": 1.00}):
        raised_back = ctx.part_world_aabb(back_cover)
    ctx.check(
        "back cover hinge raises the flat cover",
        flat_back is not None
        and raised_back is not None
        and raised_back[1][2] > flat_back[1][2] + 0.05,
        details=f"flat={flat_back}, raised={raised_back}",
    )

    lever_rest = ctx.part_world_aabb(lever_0)
    with ctx.pose({"plate_to_lever_0": 0.45}):
        lever_rotated = ctx.part_world_aabb(lever_0)
    ctx.check(
        "lever tab has a revolute throw",
        lever_rest is not None
        and lever_rotated is not None
        and abs(lever_rotated[0][2] - lever_rest[0][2]) > 0.003,
        details=f"rest={lever_rest}, rotated={lever_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
