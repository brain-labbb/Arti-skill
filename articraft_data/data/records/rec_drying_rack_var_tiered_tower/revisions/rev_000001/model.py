from __future__ import annotations

import math
from typing import Iterable

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ALUMINUM = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
DARK_METAL = Material("dark_hardware", rgba=(0.08, 0.085, 0.085, 1.0))
BLACK_PLASTIC = Material("black_plastic_feet", rgba=(0.01, 0.01, 0.01, 1.0))
PALE_PLASTIC = Material("pale_hinge_blocks", rgba=(0.78, 0.80, 0.80, 1.0))


def _as_vec(point: Iterable[float]) -> tuple[float, float, float]:
    x, y, z = point
    return (float(x), float(y), float(z))


def _cylinder_between(
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    """Closed circular tube between two local points."""
    p0 = _as_vec(start)
    p1 = _as_vec(end)
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1.0e-8:
        raise ValueError("tube endpoints must be distinct")

    geom = CylinderGeometry(radius, length, radial_segments=radial_segments, closed=True)
    direction = (vx / length, vy / length, vz / length)
    z_axis = (0.0, 0.0, 1.0)
    dot = max(-1.0, min(1.0, direction[2]))
    angle = math.acos(dot)
    axis = (-direction[1], direction[0], 0.0)
    axis_len = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
    if axis_len > 1.0e-8:
        geom.rotate((axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len), angle)
    elif direction[2] < 0.0:
        geom.rotate((1.0, 0.0, 0.0), math.pi)
    geom.translate((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return geom


def _tube_visual(
    part,
    name: str,
    start: Iterable[float],
    end: Iterable[float],
    radius: float,
    material: Material = ALUMINUM,
    *,
    radial_segments: int = 20,
) -> None:
    part.visual(
        mesh_from_geometry(
            _cylinder_between(start, end, radius, radial_segments=radial_segments),
            name,
        ),
        material=material,
        name=name,
    )


def _merged_tubes(
    segments: Iterable[tuple[tuple[float, float, float], tuple[float, float, float], float]],
    *,
    radial_segments: int = 20,
) -> MeshGeometry:
    merged = MeshGeometry()
    for start, end, radius in segments:
        merged.merge(_cylinder_between(start, end, radius, radial_segments=radial_segments))
    return merged


def _straight_frame_visual(part, name: str, segments, material=ALUMINUM) -> None:
    part.visual(
        mesh_from_geometry(_merged_tubes(segments, radial_segments=18), name),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tiered_tower_laundry_drying_rack",
        meta={
            "category_note": (
                "Tiered-tower variant of a freestanding laundry drying rack: "
                "upright rectangular tube column on four rubber feet with "
                "stacked horizontal drying shelves at different heights."
            ),
        },
    )

    rack = model.part("main_frame")

    # ── Tower proportions (real-world floor-standing rack) ──────────────
    hw = 0.28          # half-width  (X)
    hd = 0.18          # half-depth  (Y)
    post_r = 0.012     # vertical post radius
    rail_r = 0.010     # horizontal rail radius
    bar_r = 0.007      # drying-bar radius

    z_base   = 0.04    # lowest structural rail (just above feet)
    z_shelf1 = 0.40    # bottom fixed shelf
    z_shelf2 = 0.80    # middle shelf = hinged front_wing
    z_shelf3 = 1.20    # top fixed shelf
    z_cap    = 1.42    # top of the posts

    posts = [(-hw, -hd), (-hw, hd), (hw, -hd), (hw, hd)]

    # ── Merge every tube in the fixed frame into one connected mesh ────
    frame_segs: list[tuple[tuple[float, float, float],
                           tuple[float, float, float], float]] = []

    # Four vertical posts
    for x, y in posts:
        frame_segs.append(((x, y, 0.03), (x, y, z_cap), post_r))

    # Horizontal rails at five levels (base, three shelves, cap)
    for z in (z_base, z_shelf1, z_shelf2, z_shelf3, z_cap):
        frame_segs.append(((-hw,  hd, z), ( hw,  hd, z), rail_r))   # front
        frame_segs.append(((-hw, -hd, z), ( hw, -hd, z), rail_r))   # rear
        frame_segs.append(((-hw, -hd, z), (-hw,  hd, z), rail_r))   # left
        frame_segs.append((( hw, -hd, z), ( hw,  hd, z), rail_r))   # right

    # Fixed bottom-shelf drying bars (parallel, running in X)
    n_bars = 7
    for i in range(n_bars):
        t = (i + 1) / (n_bars + 1)
        y = -hd + 2.0 * hd * t
        frame_segs.append(((-hw, y, z_shelf1 + 0.015),
                           ( hw, y, z_shelf1 + 0.015), bar_r))

    # Fixed top-shelf drying bars
    for i in range(n_bars):
        t = (i + 1) / (n_bars + 1)
        y = -hd + 2.0 * hd * t
        frame_segs.append(((-hw, y, z_shelf3 + 0.015),
                           ( hw, y, z_shelf3 + 0.015), bar_r))

    _straight_frame_visual(rack, "tower_frame", frame_segs, ALUMINUM)

    # ── Rubber feet at each post base ──────────────────────────────────
    for i, (x, y) in enumerate(posts):
        rack.visual(
            Box((0.04, 0.04, 0.03)),
            origin=Origin(xyz=(x, y, 0.015)),
            material=BLACK_PLASTIC,
            name=f"rubber_foot_{i}",
        )

    # ── Hinge blocks where the front wing attaches ─────────────────────
    wing_hw = hw - 0.05   # wing half-width (narrower than frame)
    for i, x in enumerate((-wing_hw, wing_hw)):
        rack.visual(
            Box((0.04, 0.035, 0.03)),
            origin=Origin(xyz=(x, hd, z_shelf2)),
            material=PALE_PLASTIC,
            name=f"wing_hinge_block_{i}",
        )

    # ── Front wing: hinged middle drying shelf ─────────────────────────
    wing = model.part("front_wing")
    shelf_depth = 2.0 * hd - 0.04   # 0.32 m, clears the rear rails

    wing_segs: list[tuple[tuple[float, float, float],
                          tuple[float, float, float], float]] = [
        # Two side rails (hinge → back)
        ((-wing_hw, 0.0, 0.0), (-wing_hw, -shelf_depth, 0.0), 0.009),
        (( wing_hw, 0.0, 0.0), ( wing_hw, -shelf_depth, 0.0), 0.009),
        # Back rail
        ((-wing_hw, -shelf_depth, 0.0), (wing_hw, -shelf_depth, 0.0), 0.009),
    ]
    # Parallel drying bars
    n_wing_bars = 6
    for i in range(n_wing_bars):
        t = (i + 1) / (n_wing_bars + 1)
        y = -shelf_depth * t
        wing_segs.append(((-wing_hw, y, 0.0), (wing_hw, y, 0.0), 0.007))

    _straight_frame_visual(wing, "wing_tube_frame", wing_segs, ALUMINUM)

    # ── Articulation: shelf folds downward from horizontal ─────────────
    #   Origin on the front rail at mid-width, axis +X so that positive q
    #   rotates the −Y shelf edge toward −Z (folds down).
    model.articulation(
        "frame_to_front_wing",
        ArticulationType.REVOLUTE,
        parent=rack,
        child=wing,
        origin=Origin(xyz=(0.0, hd, z_shelf2)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=1.2, lower=0.0, upper=1.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rack  = object_model.get_part("main_frame")
    wing  = object_model.get_part("front_wing")
    hinge = object_model.get_articulation("frame_to_front_wing")

    # ── Intentional overlap allowances at the hinge ────────────────────
    ctx.allow_overlap(
        "front_wing", "main_frame",
        elem_a="wing_tube_frame", elem_b="tower_frame",
        reason=(
            "The wing side rails pivot at the frame shelf2 front rail, "
            "forming perpendicular T-junctions at the hinge line."
        ),
    )
    for i in range(2):
        ctx.allow_overlap(
            "front_wing", "main_frame",
            elem_a="wing_tube_frame", elem_b=f"wing_hinge_block_{i}",
            reason="Hinge block encloses the wing side-rail pivot at the hinge line.",
        )

    # ── Structure: tower with one hinged shelf ─────────────────────────
    ctx.check(
        "tiered tower rack has vertical frame and hinged shelf",
        len(object_model.parts) == 2
        and len(object_model.articulations) == 1,
        details=f"parts={len(object_model.parts)}, "
                f"joints={len(object_model.articulations)}",
    )

    # ── Vertical stacking: wing sits well above the feet ───────────────
    ctx.expect_gap(
        wing, rack,
        axis="z",
        positive_elem="wing_tube_frame",
        negative_elem="rubber_foot_0",
        min_gap=0.30,
        name="front_wing shelf is well above the rubber feet (tiered stacking)",
    )

    # ── Wing spans the rack width ──────────────────────────────────────
    ctx.expect_overlap(
        wing, rack,
        axes="x",
        min_overlap=0.35,
        elem_a="wing_tube_frame", elem_b="tower_frame",
        name="front_wing shelf spans the tower rack width",
    )

    # ── Folding motion: positive q drops the shelf downward ────────────
    rest_aabb = ctx.part_world_aabb(wing)
    with ctx.pose({hinge: 1.0}):
        folded_aabb = ctx.part_world_aabb(wing)
    ctx.check(
        "frame_to_front_wing positive motion folds the shelf downward",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] < rest_aabb[0][2] - 0.05,
        details=(
            f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, "
            f"folded_min_z={folded_aabb[0][2] if folded_aabb else None}"
        ),
    )

    # ── Proof checks for allowed overlaps ──────────────────────────────
    ctx.expect_contact(
        wing, rack,
        elem_a="wing_tube_frame", elem_b="tower_frame",
        contact_tol=0.005,
        name="wing side rails contact the frame front rail at the hinge",
    )
    for i in range(2):
        ctx.expect_contact(
            wing, rack,
            elem_a="wing_tube_frame", elem_b=f"wing_hinge_block_{i}",
            contact_tol=0.005,
            name=f"wing side rail contacts hinge block {i} at the pivot",
        )

    return ctx.report()


object_model = build_object_model()
